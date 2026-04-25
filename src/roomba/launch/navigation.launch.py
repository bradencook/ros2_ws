import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Navigation mode: bringup + slam_toolbox localization + Nav2 navigation stack.

    Uses slam_toolbox (localization mode) instead of AMCL. Full scan-to-map
    matching against the saved posegraph is noticeably tighter than AMCL's
    particle filter on this carpet where wheel odom slips a lot.

    slam_toolbox provides both /map (OccupancyGrid) and the map->odom TF, so
    nav2_bringup's localization_launch.py (map_server + AMCL) is skipped —
    we include navigation_launch.py (planner/controller/behaviors/bt) only.
    """
    roomba_dir = get_package_share_directory('roomba')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    # --- Shared bringup (TFs, roomba, IMU, EKF, RPLidar, Foxglove, action_executor) ---
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(roomba_dir, 'launch', 'bringup.launch.py')
        )
    )

    # --- slam_toolbox in localization mode (replaces AMCL + map_server) ---
    slam_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'False',
            'slam_params_file': os.path.join(
                roomba_dir, 'config', 'slam_localization_params.yaml'
            ),
        }.items()
    )

    # --- Nav2 navigation stack only (no AMCL, no map_server) ---
    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'False',
            'params_file': os.path.join(roomba_dir, 'config', 'nav2_params.yaml'),
        }.items()
    )

    return LaunchDescription([
        bringup,
        slam_localization,
        nav2_navigation,
    ])
