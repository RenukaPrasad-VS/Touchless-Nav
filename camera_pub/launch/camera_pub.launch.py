from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('camera_device', default_value=''),
        DeclareLaunchArgument('camera_index', default_value='-1'),
        DeclareLaunchArgument('max_camera_index', default_value='10'),
        DeclareLaunchArgument('image_topic', default_value='/image_raw'),
        DeclareLaunchArgument('input_topic', default_value=''),
        DeclareLaunchArgument('width', default_value='0'),
        DeclareLaunchArgument('height', default_value='0'),
        DeclareLaunchArgument('fps', default_value='30.0'),
        DeclareLaunchArgument('mirror', default_value='true'),
        DeclareLaunchArgument('mirror_input', default_value='false'),
    ]
    camera_node = Node(
        package='camera_pub',
        executable='camera_publisher.py',
        name='camera_publisher',
        output='screen',
        parameters=[{
            'camera_device': LaunchConfiguration('camera_device'),
            'camera_index': ParameterValue(
                LaunchConfiguration('camera_index'), value_type=int),
            'max_camera_index': ParameterValue(
                LaunchConfiguration('max_camera_index'), value_type=int),
            'image_topic': LaunchConfiguration('image_topic'),
            'input_topic': LaunchConfiguration('input_topic'),
            'width': ParameterValue(LaunchConfiguration('width'), value_type=int),
            'height': ParameterValue(LaunchConfiguration('height'), value_type=int),
            'fps': ParameterValue(LaunchConfiguration('fps'), value_type=float),
            'mirror': ParameterValue(LaunchConfiguration('mirror'), value_type=bool),
            'mirror_input': ParameterValue(
                LaunchConfiguration('mirror_input'), value_type=bool),
        }],
    )

    return LaunchDescription(arguments + [camera_node])
