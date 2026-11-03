#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
import os


from map_process import MapProcessor
from handclick import HandClickDetector


class MapOverlayNode(Node):
    """ROS 2 node that overlays map and reacts to touchless tap gestures."""

    def __init__(self):
        super().__init__('map_overlay_node')
        self.bridge = CvBridge()

        pkg_share = get_package_share_directory('turtlebot_nav2')
        map_path = os.path.join(pkg_share, 'maps', 'tb3_world.pgm')
        yaml_path = os.path.join(pkg_share, 'maps', 'tb3_world.yaml')

        self.declare_parameter("map_path", map_path)
        self.declare_parameter("yaml_path", yaml_path)
        self.declare_parameter("alpha", 0.6)
        self.declare_parameter("image_topic", "/image_raw")

        map_path = self.get_parameter("map_path").get_parameter_value().string_value
        yaml_path = self.get_parameter("yaml_path").get_parameter_value().string_value
        self.alpha = self.get_parameter("alpha").get_parameter_value().double_value
        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value

        self.processor = MapProcessor(map_path, yaml_path)
        self.overlay = self.processor.overlay
        # self.overlay = cv2.flip(self.overlay, 0)
        self.window_name = "Touchless Map Overlay"
        self.map_info = {
            "resolution": self.processor.resolution,
            "origin": self.processor.origin,
            "height": self.processor.height,
            "width": self.processor.width
        }


        self.detector = HandClickDetector()
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_request_pending = False
        self.active_goal_handle = None
        self.get_logger().info("📍 Ready to send Nav2 NavigateToPose goals")

        self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.get_logger().info(f"🖼️ Subscribed to {image_topic}")
        self.get_logger().info("✋ Use hand tap gesture to click on image (ESC to exit).")


    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        blended = self._blend_frame_with_overlay(frame)
        processed_frame, coords = self.detector.process_frame(blended)

        if coords:
            x, y = coords
            wx, wy = self.processor.image_to_world(x, y)
            self.get_logger().info(f"🟢 Tap at pixel ({x},{y}) → world ({wx:.3f},{wy:.3f}) m")
            cv2.circle(processed_frame, (x, y), 10, (0, 255, 0), 2)

            self._send_navigation_goal(wx, wy)


        cv2.imshow(self.window_name, processed_frame)

        if cv2.waitKey(1) & 0xFF == 27:
            self.get_logger().info("🛑 Exiting overlay viewer")
            rclpy.shutdown()

    def _blend_frame_with_overlay(self, frame):
        fh, fw = frame.shape[:2]
        mh, mw = self.overlay.shape[:2]
        target_aspect = mw / mh

        new_w = int(fh * target_aspect)
        if new_w <= fw:
            x_start = (fw - new_w) // 2
            frame_cropped = frame[:, x_start:x_start + new_w]
        else:
            new_h = int(fw / target_aspect)
            y_start = (fh - new_h) // 2
            frame_cropped = frame[y_start:y_start + new_h, :]

        frame_resized = cv2.resize(frame_cropped, (mw, mh))
        gray_map = cv2.cvtColor(self.overlay, cv2.COLOR_BGR2GRAY)
        mask = gray_map < 250
        blended = frame_resized.copy()
        blended[mask] = cv2.addWeighted(frame_resized[mask], 1 - self.alpha,
                                        self.overlay[mask], self.alpha, 0)
        return blended

    def _send_navigation_goal(self, wx, wy):
        if self.goal_request_pending:
            self.get_logger().warning("A Nav2 goal request is already pending; tap again shortly.")
            return
        if not self.nav_client.server_is_ready():
            self.get_logger().warning("Nav2 navigate_to_pose action server is not ready; no goal sent.")
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(wx)
        goal.pose.pose.position.y = float(wy)
        goal.pose.pose.orientation.w = 1.0

        self.goal_request_pending = True
        future = self.nav_client.send_goal_async(goal, feedback_callback=self._on_nav_feedback)
        future.add_done_callback(self._on_goal_response)
        self.get_logger().info(f"🎯 Requesting Nav2 goal: ({wx:.2f}, {wy:.2f})")

    def _on_goal_response(self, future):
        self.goal_request_pending = False
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"Nav2 goal request failed: {exc}")
            return
        if not goal_handle.accepted:
            self.get_logger().warning("Nav2 rejected the clicked goal.")
            return

        self.active_goal_handle = goal_handle
        self.get_logger().info("Nav2 accepted the goal.")
        goal_handle.get_result_async().add_done_callback(
            lambda result_future, handle=goal_handle:
                self._on_nav_result(result_future, handle))

    def _on_nav_feedback(self, _feedback):
        # Feedback is available here for future UI progress display.
        pass

    def _on_nav_result(self, future, goal_handle):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to get Nav2 result: {exc}")
            return
        if self.active_goal_handle is goal_handle:
            self.active_goal_handle = None
        if response.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Nav2 reached the clicked goal.")
        elif response.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("Nav2 goal was canceled.")
        else:
            self.get_logger().warning(f"Nav2 goal finished with status {response.status}.")

    def destroy_node(self):
        self.detector.close()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MapOverlayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
