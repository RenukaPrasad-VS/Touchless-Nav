"""Start the VLA demo world, TurtleBot3, and Gazebo ROS bridges."""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_share = get_package_share_directory('turtlebot_description')
    vla_share = get_package_share_directory('vla_nav')
    robot_description = xacro.process_file(
        os.path.join(description_share, 'urdf', 'turtlebot.urdf.xacro')).toxml()
    world_file = os.path.join(vla_share, 'worlds', 'vla_objects.sdf')

    model_paths = [
        description_share,
        os.path.join(vla_share, 'models'),
        os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''),
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    ]
    resource_path = os.pathsep.join(dict.fromkeys(filter(None, model_paths)))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py']),
        launch_arguments={'gz_args': ['-r -v 4 ', world_file]}.items(),
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', 'empty',
            '-string', robot_description,
            '-name', 'turtlebot3',
            '-allow_renaming', 'true',
            '-x', '-2.0', '-y', '-0.5', '-z', '0.01',
            '-R', '0.0', '-P', '0.0', '-Y', '0.0',
        ],
    )
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/odom/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
        ],
        remappings=[('/odom/tf', 'tf')],
        output='screen',
    )

    return LaunchDescription([
        # Humble commonly runs Gazebo Fortress (IGN_*); newer Gazebo uses GZ_*.
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_path),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
    ])
