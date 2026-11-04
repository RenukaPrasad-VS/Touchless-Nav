from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    defaults = {'min_gesture_confidence': '0.65', 'stable_frames': '3',
                'min_hand_detection_confidence': '0.45', 'min_hand_presence_confidence': '0.45',
                'min_tracking_confidence': '0.5'}
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()] +
        [Node(package='gesture_control', executable='gc_node.py', name='gesture_teleop_node',
              output='screen', parameters=[{k: LaunchConfiguration(k) for k in defaults}])]
    )
