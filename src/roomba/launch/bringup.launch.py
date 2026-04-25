import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Shared bringup: static TFs, roomba driver, IMU, EKF, RPLidar, Foxglove."""
    roomba_dir = get_package_share_directory('roomba')
    rplidar_dir = get_package_share_directory('rplidar_ros')

    # --- Static transforms ---
    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser',
        arguments=[
            '--x', '0.02', '--y', '0', '--z', '0.2',
            '--roll', '0', '--pitch', '0', '--yaw', '3.14159',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser',
        ]
    )

    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera',
        arguments=[
            '--x', '0.05', '--y', '0', '--z', '0.1',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link',
        ]
    )

    # BNO08x mounted just below the lidar, axes aligned with base_link (X fwd, Y left, Z up).
    # Translation is approximate — adjust if you refine the mounting.
    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu',
        arguments=[
            '--x', '0.02', '--y', '0', '--z', '0.18',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'imu_link',
        ]
    )

    # --- Core robot driver (publishes /wheel/odometry, no TF) ---
    roomba_node = Node(
        package='roomba',
        executable='roomba_node',
        name='roomba_node',
        output='screen',
    )

    # --- IMU driver (publishes /imu/data) ---
    imu_node = Node(
        package='roomba',
        executable='imu_node',
        name='imu_node',
        output='screen',
    )

    # --- Bump + cliff sensors -> costmap obstacles (fills the lidar blind spot below 20cm) ---
    bump_obstacle_node = Node(
        package='roomba',
        executable='bump_obstacle_node',
        name='bump_obstacle',
        output='screen',
    )

    # --- High-level action executor (goto/look_around/save_location) ---
    # Writes locations.yaml back to the source tree so saves survive rebuilds.
    action_executor_node = Node(
        package='roomba',
        executable='action_executor',
        name='action_executor',
        output='screen',
        parameters=[{
            'locations_file': os.path.expanduser(
                '~/ros2_ws/src/roomba/config/locations.yaml'
            ),
        }],
    )

    # --- Sensor fusion: /wheel/odometry + /imu/data -> /odom + odom->base_link TF ---
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(roomba_dir, 'config', 'ekf.yaml')],
        remappings=[('/odometry/filtered', '/odom')],
    )

    # --- Lidar ---
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rplidar_dir, 'launch', 'rplidar_c1_custom.launch.py')
        )
    )

    # --- Visualization ---
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
    )

    return LaunchDescription([
        static_tf_laser,
        static_tf_camera,
        static_tf_imu,
        roomba_node,
        imu_node,
        bump_obstacle_node,
        action_executor_node,
        ekf_node,
        rplidar_launch,
        foxglove_bridge,
    ])
