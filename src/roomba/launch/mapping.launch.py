import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Mapping mode: bringup + SLAM Toolbox (online async)."""
    roomba_dir = get_package_share_directory('roomba')

    # --- Shared bringup (static TFs, roomba driver, RPLidar, Foxglove) ---
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(roomba_dir, 'launch', 'bringup.launch.py')
        )
    )

    # --- SLAM Toolbox ---
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch', 'online_async_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'False',
            'slam_params_file': os.path.join(
                roomba_dir, 'config', 'slam_params.yaml'
            ),
        }.items()
    )

    return LaunchDescription([
        bringup,
        slam_toolbox,
    ])
