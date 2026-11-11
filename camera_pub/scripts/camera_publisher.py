#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('input_topic', '')
        self.declare_parameter('camera_device', '')
        self.declare_parameter('camera_index', -1)
        self.declare_parameter('max_camera_index', 10)
        self.declare_parameter('width', 0)
        self.declare_parameter('height', 0)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('mirror', True)
        self.declare_parameter('mirror_input', False)
        self.publisher_ = self.create_publisher(
            Image, str(self.get_parameter('image_topic').value), 10)
        self.bridge = CvBridge()
        self.mirror = bool(self.get_parameter('mirror').value)
        input_topic = str(self.get_parameter('input_topic').value).strip()
        self.cap = None
        self.input_subscription = None
        self.timer = None
        if input_topic:
            output_topic = str(self.get_parameter('image_topic').value)
            if input_topic == output_topic:
                raise ValueError('input_topic and image_topic must be different')
            self.input_subscription = self.create_subscription(
                Image, input_topic, self.input_callback, qos_profile_sensor_data)
            self.get_logger().info(
                f'Relaying camera images from {input_topic} to {output_topic}')
        else:
            self.cap = self.open_camera()
            fps = float(self.get_parameter('fps').value)
            self.timer = self.create_timer(
                1.0 / max(fps, 1.0), self.timer_callback)

    def input_callback(self, msg):
        if bool(self.get_parameter('mirror_input').value):
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            mirrored_msg = self.bridge.cv2_to_imgmsg(
                cv2.flip(frame, 1), encoding='bgr8')
            mirrored_msg.header = msg.header
            msg = mirrored_msg
        self.publisher_.publish(msg)

    def open_camera(self):
        device = str(self.get_parameter('camera_device').value).strip()
        camera_index = int(self.get_parameter('camera_index').value)
        max_camera_index = int(self.get_parameter('max_camera_index').value)

        sources = [device] if device else (
            [camera_index] if camera_index >= 0
            else list(range(max(0, max_camera_index) + 1)))
        for source in sources:
            capture = cv2.VideoCapture(source, cv2.CAP_ANY)
            if capture.isOpened():
                width = int(self.get_parameter('width').value)
                height = int(self.get_parameter('height').value)
                if width > 0:
                    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                if height > 0:
                    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                self.get_logger().info(
                    f'Opened camera {source}; publishing on '
                    f'{self.get_parameter("image_topic").value}')
                return capture
            capture.release()

        self.get_logger().error(
            f'Could not open camera source(s): {sources}. Set camera_device '
            'or camera_index to select a camera explicitly.')
        return None

    def timer_callback(self):
        if self.cap is None:
            return
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to read frame from camera')
            return
        if self.mirror:
            frame = cv2.flip(frame, 1)
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    finally:
        if node.cap is not None:
            node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
