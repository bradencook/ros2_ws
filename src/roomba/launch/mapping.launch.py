import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Directories
    roomba_dir = get_package_share_directory('roomba')
    rplidar_dir = get_package_share_directory('rplidar_ros')

    # Component Launches
    webcam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(roomba_dir, 'launch', 'webcam.launch.py')
        )
    )

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rplidar_dir, 'launch', 'rplidar_c1_custom.launch.py')
        )
    )

    # Core Nodes
    roomba_node = Node(
        package='roomba',
        executable='roomba_node',
        name='roomba_node',
        output='screen'
    )

    # SLAM & Visualization
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen'
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'odom_frame': 'odom',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'scan_topic': '/scan',
            'use_sim_time': False,
        }]
    )

    # Static Transforms (Modify Z-heights or frames as needed for your physical robot)
    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser',
        arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'laser_frame']
    )
    
    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera',
        arguments=['0.1', '0', '0.1', '0', '0', '0', 'base_link', 'camera_link']
    )

    return LaunchDescription([
        roomba_node,
        rplidar_launch,
        webcam_launch,
        foxglove_bridge,
        slam_toolbox,
        static_tf_laser,
        static_tf_camera
    ])
