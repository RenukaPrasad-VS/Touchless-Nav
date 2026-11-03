"""Vision-language object grounding with depth and Nav2 execution."""

import base64
import io
import json
import math
import queue
import re
import struct
import threading
import time
import urllib.request

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


class VLAObjectNavNode(Node):
    """Ground a language-named object in RGB-D, then ask Nav2 to approach it."""

    def __init__(self):
        super().__init__('vla_object_nav')
        self.declare_parameter('instruction_topic', '/voice/recognized_text')
        self.declare_parameter('image_topic', '/camera/image')
        self.declare_parameter('depth_topic', '/camera/depth_image')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('ollama_url', 'http://localhost:11434/api/chat')
        self.declare_parameter('model', 'qwen3-vl:4b')
        self.declare_parameter('navigate_action', 'navigate_to_pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        # Depth is sampled on the visible chair surface, so this is clearance
        # from that surface (rather than from the object's center). A modest
        # default brings the robot close while leaving room for its footprint.
        self.declare_parameter('standoff_m', 0.15)
        self.declare_parameter('max_depth_m', 5.0)
        self.declare_parameter('min_depth_m', 0.15)
        self.declare_parameter('max_image_age_sec', 1.5)
        self.declare_parameter('camera_frame_uses_optical_convention', False)

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.ollama_url = str(self.get_parameter('ollama_url').value)
        self.model = str(self.get_parameter('model').value)
        self.standoff = float(self.get_parameter('standoff_m').value)
        self.min_depth = float(self.get_parameter('min_depth_m').value)
        self.max_depth = float(self.get_parameter('max_depth_m').value)
        self.max_image_age = float(self.get_parameter('max_image_age_sec').value)
        self.camera_frame_uses_optical_convention = bool(
            self.get_parameter('camera_frame_uses_optical_convention').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(
            self, NavigateToPose,
            str(self.get_parameter('navigate_action').value))

        camera_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT)
        self.rgb = None
        self.depth = None
        self.camera_info = None
        self.rgb_received = 0.0
        self.depth_received = 0.0
        self.info_received = 0.0
        self.result_queue = queue.Queue()
        self.processing = False
        self.request_generation = 0
        self.goal_pending = False
        self.goal_handle = None
        self.cancel_requested = False

        self.create_subscription(
            Image, str(self.get_parameter('image_topic').value),
            self._on_rgb, camera_qos)
        self.create_subscription(
            Image, str(self.get_parameter('depth_topic').value),
            self._on_depth, camera_qos)
        self.create_subscription(
            CameraInfo, str(self.get_parameter('camera_info_topic').value),
            self._on_camera_info, camera_qos)
        self.create_subscription(
            String, str(self.get_parameter('instruction_topic').value),
            self._on_instruction, 10)
        self.create_timer(0.05, self._drain_results)
        self.get_logger().info(
            f'VLA object navigation ready; model={self.model}. '
            'Try: "go to the chair".')

    def _on_rgb(self, msg):
        self.rgb = msg
        self.rgb_received = time.monotonic()

    def _on_depth(self, msg):
        self.depth = msg
        self.depth_received = time.monotonic()

    def _on_camera_info(self, msg):
        self.camera_info = msg
        self.info_received = time.monotonic()

    def _on_instruction(self, msg):
        instruction = msg.data.strip()
        if not instruction:
            return
        if re.search(r'\b(stop|cancel)\b', instruction, re.IGNORECASE):
            self._cancel_goal()
            return
        if self.goal_pending or self.processing:
            self.get_logger().warning('Busy with a VLA request or Nav2 goal; cancel first')
            return
        if self.rgb is None or time.monotonic() - self.rgb_received > self.max_image_age:
            self.get_logger().error('No fresh RGB frame; check image_topic and camera publisher')
            return
        if self.depth is None or self.camera_info is None:
            self.get_logger().error('Waiting for depth image and camera info')
            return
        now = time.monotonic()
        if (now - self.depth_received > self.max_image_age or
                now - self.info_received > self.max_image_age):
            self.get_logger().error('Depth image or CameraInfo is stale')
            return

        rgb = self.rgb
        depth = self.depth
        camera_info = self.camera_info
        try:
            image_bytes = self._encode_jpeg(rgb)
        except Exception as exc:
            self.get_logger().error(f'Could not encode camera image: {exc}')
            return

        self.processing = True
        self.request_generation += 1
        generation = self.request_generation
        started = time.monotonic()
        threading.Thread(
            target=self._infer_object,
            args=(instruction, image_bytes, started, rgb, depth, camera_info, generation),
            daemon=True).start()

    @staticmethod
    def _encode_jpeg(msg):
        from PIL import Image as PILImage

        formats = {
            'rgb8': ('RGB', 'RGB'),
            'bgr8': ('RGB', 'BGR'),
            'rgba8': ('RGBA', 'RGBA'),
            'bgra8': ('RGBA', 'BGRA'),
            'mono8': ('L', 'L'),
        }
        if msg.encoding not in formats:
            raise ValueError(f'unsupported RGB encoding {msg.encoding!r}')
        mode, raw_mode = formats[msg.encoding]
        image = PILImage.frombytes(
            mode, (msg.width, msg.height), bytes(msg.data),
            'raw', raw_mode, msg.step, 1).convert('RGB')
        image.thumbnail((640, 480))
        output = io.BytesIO()
        image.save(output, format='JPEG', quality=82)
        return output.getvalue()

    def _infer_object(
            self, instruction, image_bytes, started, rgb, depth, camera_info, generation):
        schema = {
            'type': 'object',
            'properties': {
                'action': {'type': 'string', 'enum': ['navigate', 'not_found', 'ignore']},
                'target': {'type': 'string'},
                'bbox': {
                    'type': 'array', 'items': {'type': 'integer'},
                    'minItems': 4, 'maxItems': 4,
                },
            },
            'required': ['action', 'target', 'bbox'],
        }
        prompt = (
            'You ground navigation instructions in the supplied robot camera image. '
            'Find the specific named physical object requested by the user. '
            'Return bbox [xmin,ymin,xmax,ymax] in normalized integer coordinates '
            'from 0 to 1000, measured from the full image top-left. '
            'Use action navigate only when that object is visibly identifiable. '
            'Otherwise use not_found. Never invent an object or a location. '
            'This output is only a target candidate; a robot navigation planner '
            'and depth sensor will determine whether it is reachable. '
            f'User instruction: {instruction}')
        payload = {
            'model': self.model,
            'stream': False,
            'format': schema,
            'messages': [{
                'role': 'user',
                'content': prompt,
                'images': [base64.b64encode(image_bytes).decode('ascii')],
            }],
            'options': {'temperature': 0},
        }
        try:
            request = urllib.request.Request(
                self.ollama_url,
                data=json.dumps(payload).encode('utf-8'),
                headers={'Content-Type': 'application/json'},
                method='POST')
            with urllib.request.urlopen(request, timeout=90) as response:
                response_data = json.loads(response.read().decode('utf-8'))
            result = json.loads(response_data['message']['content'])
            self.result_queue.put(
                ('inference', result, started, None, rgb, depth, camera_info, generation))
        except Exception as exc:
            self.result_queue.put(
                ('inference', None, started, str(exc), rgb, depth, camera_info, generation))

    def _drain_results(self):
        while not self.result_queue.empty():
            kind, result, started, error, rgb, depth, camera_info, generation = \
                self.result_queue.get_nowait()
            if kind != 'inference':
                continue
            if generation != self.request_generation:
                continue
            self.processing = False
            elapsed = time.monotonic() - started
            if error:
                self.get_logger().error(
                    f'VLM inference failed after {elapsed:.2f}s: {error}. '
                    'Is Ollama running with the configured model?')
                continue
            if not isinstance(result, dict):
                self.get_logger().error('VLM response must be a JSON object')
                continue
            action = result.get('action')
            if action != 'navigate':
                self.get_logger().warning(
                    f'VLM did not find a safe target (action={action}, '
                    f'target={result.get("target", "")!r})')
                continue
            try:
                bbox = [int(v) for v in result['bbox']]
                if len(bbox) != 4 or not all(0 <= v <= 1000 for v in bbox):
                    raise ValueError('bbox values must be four integers from 0 to 1000')
                if bbox[2] <= bbox[0] or bbox[3] <= bbox[1]:
                    raise ValueError('bbox has zero or negative size')
                self._navigate_to_bbox(
                    bbox, str(result.get('target', 'object')), elapsed,
                    rgb, depth, camera_info)
            except (KeyError, TypeError, ValueError) as exc:
                self.get_logger().error(f'Invalid VLM target response: {exc}')

    def _navigate_to_bbox(self, bbox, label, inference_sec, rgb, depth_msg, info):
        if not self.nav_client.wait_for_server(timeout_sec=0.25):
            self.get_logger().error('Nav2 navigate_to_pose action is unavailable')
            return
        image_width, image_height = rgb.width, rgb.height
        xmin, ymin, xmax, ymax = bbox
        # Use a point toward the lower part of the box. The box center often
        # lands on a raised seat/backrest, while the lower visible surface is
        # a better estimate of where the object meets the floor.
        u = int(((xmin + xmax) / 2.0) * image_width / 1000.0)
        v = int((ymin + 0.8 * (ymax - ymin)) * image_height / 1000.0)
        u = min(max(u, 0), image_width - 1)
        v = min(max(v, 0), image_height - 1)

        if depth_msg.width != image_width or depth_msg.height != image_height:
            self.get_logger().error('RGB and depth dimensions differ; use aligned RGB-D topics')
            return
        try:
            depth_m = self._sample_depth(depth_msg, u, v)
        except (ValueError, struct.error) as exc:
            self.get_logger().error(f'Could not read depth at target: {exc}')
            return
        if not math.isfinite(depth_m) or not self.min_depth <= depth_m <= self.max_depth:
            self.get_logger().error(
                f'No valid target depth near pixel ({u},{v}); got {depth_m!r} m')
            return
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        if fx <= 0.0 or fy <= 0.0:
            self.get_logger().error('Invalid camera intrinsics in CameraInfo.K')
            return

        # REP-103 optical frame: +X right, +Y down, +Z forward.
        camera_point = ((u - cx) * depth_m / fx,
                        (v - cy) * depth_m / fy,
                        depth_m)
        camera_frame = info.header.frame_id or depth_msg.header.frame_id
        if not self.camera_frame_uses_optical_convention:
            # This repo's Gazebo URDF labels the camera frame optical but gives
            # camera_optical_joint zero rotation. Convert REP-103 optical
            # coordinates (right, down, forward) to camera_link coordinates
            # (forward, left, up) before applying that frame's TF.
            camera_point = (camera_point[2], -camera_point[0], -camera_point[1])
        try:
            sensor_stamp = depth_msg.header.stamp
            transform_time = (Time.from_msg(
                sensor_stamp, clock_type=self.get_clock().clock_type)
                if sensor_stamp.sec or sensor_stamp.nanosec
                else Time(clock_type=self.get_clock().clock_type))
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, camera_frame,
                transform_time,
                timeout=Duration(seconds=0.15))
            target_x, target_y = self._transform_point(camera_point, transform)
            robot = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame,
                Time(clock_type=self.get_clock().clock_type),
                timeout=Duration(seconds=0.15))
        except TransformException as exc:
            self.get_logger().error(f'Cannot transform camera target into map: {exc}')
            return

        robot_x = robot.transform.translation.x
        robot_y = robot.transform.translation.y
        dx, dy = target_x - robot_x, target_y - robot_y
        range_xy = math.hypot(dx, dy)
        if range_xy <= self.standoff:
            self.get_logger().warning(f'{label} is already within approach distance')
            return
        ux, uy = dx / range_xy, dy / range_xy
        goal_x = target_x - ux * self.standoff
        goal_y = target_y - uy * self.standoff
        # Place the robot between its current position and the visible object
        # surface, then face it toward the object.
        yaw = math.atan2(dy, dx)

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = goal_x
        goal.pose.pose.position.y = goal_y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.goal_pending = True
        self.cancel_requested = False
        self.get_logger().info(
            f'VLA grounded "{label}" at ({target_x:.2f}, {target_y:.2f}) m; '
            f'front approach goal=({goal_x:.2f}, {goal_y:.2f}), '
            f'inference={inference_sec:.2f}s')
        self.nav_client.send_goal_async(goal).add_done_callback(self._on_goal_response)

    def _sample_depth(self, msg, u, v):
        if msg.encoding == '32FC1':
            fmt, pixel_bytes, scale = 'f', 4, 1.0
        elif msg.encoding == '16UC1' or msg.encoding == 'mono16':
            fmt, pixel_bytes, scale = 'H', 2, 0.001
        else:
            raise ValueError(f'unsupported depth encoding {msg.encoding!r}')
        endian = '>' if msg.is_bigendian else '<'
        samples = []
        radius = 3
        data = memoryview(msg.data)
        for y in range(max(0, v - radius), min(msg.height, v + radius + 1)):
            for x in range(max(0, u - radius), min(msg.width, u + radius + 1)):
                offset = y * msg.step + x * pixel_bytes
                value = struct.unpack_from(endian + fmt, data, offset)[0] * scale
                if math.isfinite(value) and value > 0:
                    samples.append(value)
        if not samples:
            return float('nan')
        samples.sort()
        return samples[len(samples) // 2]

    @staticmethod
    def _transform_point(point, transform):
        t = transform.transform.translation
        q = transform.transform.rotation
        # Quaternion-vector rotation, without NumPy.
        vx, vy, vz = point
        tx = 2.0 * (q.y * vz - q.z * vy)
        ty = 2.0 * (q.z * vx - q.x * vz)
        tz = 2.0 * (q.x * vy - q.y * vx)
        rx = vx + q.w * tx + (q.y * tz - q.z * ty)
        ry = vy + q.w * ty + (q.z * tx - q.x * tz)
        rz = vz + q.w * tz + (q.x * ty - q.y * tx)
        return t.x + rx, t.y + ry

    def _cancel_goal(self):
        if self.processing:
            self.request_generation += 1
            self.processing = False
            self.get_logger().info('Canceled the in-flight VLM request')
        if self.goal_handle is not None:
            self.cancel_requested = True
            self.goal_handle.cancel_goal_async()
            self.get_logger().info('Requested Nav2 goal cancellation')
        elif self.goal_pending:
            self.cancel_requested = True
            self.get_logger().info('Will cancel when Nav2 acknowledges the goal')
        else:
            self.get_logger().info('No active Nav2 goal')

    def _on_goal_response(self, future):
        try:
            self.goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'Nav2 goal request failed: {exc}')
            self.goal_pending = False
            return
        if not self.goal_handle.accepted:
            self.get_logger().warning('Nav2 rejected the object approach goal')
            self.goal_handle = None
            self.goal_pending = False
            return
        if self.cancel_requested:
            self.goal_handle.cancel_goal_async()
        self.goal_handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        try:
            result = future.result()
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Nav2 reached the object approach pose')
            elif result.status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info('Nav2 goal canceled')
            else:
                self.get_logger().warning(f'Nav2 finished with status {result.status}')
        except Exception as exc:
            self.get_logger().error(f'Nav2 result failed: {exc}')
        self.goal_handle = None
        self.goal_pending = False
        self.cancel_requested = False

    def destroy_node(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VLAObjectNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
