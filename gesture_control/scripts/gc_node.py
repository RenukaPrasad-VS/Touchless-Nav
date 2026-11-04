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
        defaults = {"min_gesture_confidence": 0.65, "stable_frames": 3, "stale_timeout": 0.5,
                    "min_hand_detection_confidence": 0.45, "min_hand_presence_confidence": 0.45,
                    "min_tracking_confidence": 0.5}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        param = lambda name: self.get_parameter(name).value
        self.min_gesture_confidence = float(param("min_gesture_confidence"))
        self.stable_frames = min(5, max(1, int(param("stable_frames"))))
        self.stale_timeout = float(param("stale_timeout"))
        self._results_lock = threading.Lock()
        self._recent_gestures = deque(maxlen=5)
        self._recent_scores = deque(maxlen=5)
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
            **{name: float(param(name)) for name in (
                "min_hand_detection_confidence", "min_hand_presence_confidence", "min_tracking_confidence"
            )},
            result_callback=self.print_result,
        )

        self.recognizer = GestureRecognizer.create_from_options(self.options)
        self.get_logger().info("✅ Gesture teleoperation node started!")

    # --- MediaPipe result callback ---
    def print_result(self, result: GestureRecognizerResult, output_image: mp.Image, timestamp_ms: int):
        with self._results_lock:
            self._last_result_monotonic = time.monotonic()
            self._stale_stop_sent = False

            category = result.gestures[0][0] if result.gestures and result.gestures[0] else None
            score = float(category.score) if category else 0.0
            label = category.category_name if score >= self.min_gesture_confidence else None
            self._recent_gestures.append(label)
            self._recent_scores.append(score)

            # A missing hand or low-confidence prediction stops immediately.
            if label is None:
                self._recent_gestures.clear()
                self._recent_scores.clear()
                self._publish_action("Stop")
                return

            # Weight votes by the model score so weak classifications do not
            # outweigh several clearer observations of another gesture.
            votes = {name: sum(s for g, s in zip(self._recent_gestures, self._recent_scores) if g == name)
                     for name in set(self._recent_gestures) if name}
            if votes:
                winner = max(votes, key=votes.get)
                if self._recent_gestures.count(winner) >= self.stable_frames:
                    self._publish_action(winner)

    def _publish_action(self, gesture):
        actions = {"Thumb_Up": ("Forward", self.linear_speed, 0.0),
                   "Thumb_Down": ("Backward", -self.linear_speed, 0.0),
                   "Victory": ("Turn Left", 0.0, self.angular_speed),
                   "Pointing_Up": ("Turn Right", 0.0, -self.angular_speed)}
        self.current_action, linear, angular = actions.get(gesture, ("Stop", 0.0, 0.0))
        twist = Twist()
        twist.linear.x, twist.angular.z = linear, angular
        self.cmd_pub.publish(twist)

    def _check_stale_result(self):
        with self._results_lock:
            stale = time.monotonic() - self._last_result_monotonic > self.stale_timeout
            if stale and not self._stale_stop_sent:
                self._recent_gestures.clear()
                self._recent_scores.clear()
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
