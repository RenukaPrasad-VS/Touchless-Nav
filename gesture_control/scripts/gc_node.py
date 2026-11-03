#!/usr/bin/env python3
import os
import threading
import time
from collections import deque
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge
import mediapipe as mp
from ament_index_python.packages import get_package_share_directory


# --- MediaPipe setup ---
BaseOptions = mp.tasks.BaseOptions
GestureRecognizer = mp.tasks.vision.GestureRecognizer
GestureRecognizerOptions = mp.tasks.vision.GestureRecognizerOptions
GestureRecognizerResult = mp.tasks.vision.GestureRecognizerResult
VisionRunningMode = mp.tasks.vision.RunningMode


class GestureTeleop(Node):
    def __init__(self):
        super().__init__("gesture_teleop_node")

        # --- ROS interfaces ---
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, "/image_raw", self.image_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # --- Motion parameters ---
        self.linear_speed = 0.2
        self.angular_speed = 0.4
        self.current_action = "Waiting..."
        self.declare_parameter("min_gesture_confidence", 0.65)
        self.declare_parameter("stable_frames", 3)
        self.declare_parameter("stale_timeout", 0.5)
        self.min_gesture_confidence = float(self.get_parameter("min_gesture_confidence").value)
        self.stable_frames = min(5, max(1, int(self.get_parameter("stable_frames").value)))
        self.stale_timeout = float(self.get_parameter("stale_timeout").value)
        self._results_lock = threading.Lock()
        self._recent_gestures = deque(maxlen=5)
        self._last_result_monotonic = time.monotonic()
        self._last_timestamp_ms = 0
        self._stale_stop_sent = False
        self._watchdog = self.create_timer(0.1, self._check_stale_result)

        # --- Locate model file from package share dir ---
        pkg_share = get_package_share_directory("gesture_control")
        model_path = os.path.join(pkg_share, "model", "gesture_recognizer.task")

        if not os.path.exists(model_path):
            self.get_logger().error(f"❌ Model not found at: {model_path}")
            raise FileNotFoundError(model_path)

        # --- Setup MediaPipe Gesture Recognizer ---
        self.options = GestureRecognizerOptions(
            base_options=BaseOptions(model_asset_path=model_path),
            running_mode=VisionRunningMode.LIVE_STREAM,
            result_callback=self.print_result,
        )

        self.recognizer = GestureRecognizer.create_from_options(self.options)
        self.get_logger().info("✅ Gesture teleoperation node started!")

    # --- MediaPipe result callback ---
    def print_result(self, result: GestureRecognizerResult, output_image: mp.Image, timestamp_ms: int):
        with self._results_lock:
            self._last_result_monotonic = time.monotonic()
            self._stale_stop_sent = False

            label = None
            if result.gestures and result.gestures[0]:
                category = result.gestures[0][0]
                if category.score >= self.min_gesture_confidence:
                    label = category.category_name
            self._recent_gestures.append(label)

            # Any uncertain or missing detection stops motion immediately.
            if label is None:
                self._recent_gestures.clear()
                self._publish_action("Stop")
                return

            votes = sum(candidate == label for candidate in self._recent_gestures)
            if votes >= self.stable_frames:
                self._publish_action(label)

    def _publish_action(self, gesture):
        twist = Twist()
        if gesture == "Thumb_Up":
            self.current_action = "Forward"
            twist.linear.x = self.linear_speed
        elif gesture == "Thumb_Down":
            self.current_action = "Backward"
            twist.linear.x = -self.linear_speed
        elif gesture == "Victory":
            self.current_action = "Turn Left"
            twist.angular.z = self.angular_speed
        elif gesture == "Pointing_Up":
            self.current_action = "Turn Right"
            twist.angular.z = -self.angular_speed
        elif gesture == "Open_Palm":
            self.current_action = "Stop"
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            self.current_action = "Stop"

        self.cmd_pub.publish(twist)

    def _check_stale_result(self):
        with self._results_lock:
            stale = time.monotonic() - self._last_result_monotonic > self.stale_timeout
            if stale and not self._stale_stop_sent:
                self._recent_gestures.clear()
                self._publish_action("Stop")
                self._stale_stop_sent = True

    # --- ROS Image callback ---
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            timestamp_ms = time.monotonic_ns() // 1_000_000
            timestamp_ms = max(timestamp_ms, self._last_timestamp_ms + 1)
            self._last_timestamp_ms = timestamp_ms

            self.recognizer.recognize_async(mp_image, timestamp_ms)

            cv2.putText(frame, f"Gesture: {self.current_action}", (30, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 255, 0), 3)
            cv2.imshow("Gesture Teleop", frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Image callback error: {e}")

    def destroy_node(self):
        self.recognizer.close()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GestureTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Gesture teleop stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
