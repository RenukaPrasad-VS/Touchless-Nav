from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('vla_nav'), 'config', 'vla_nav.yaml')
    node = Node(
        package='vla_nav',
        executable='vla_object_nav',
        name='vla_object_nav',
        output='screen',
        parameters=[config_file],
    )
    return LaunchDescription([node])
