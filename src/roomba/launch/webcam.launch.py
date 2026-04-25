from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    # ustreamer: lightweight MJPG-HTTP streamer, reads hardware MJPG directly.
    # Near-zero CPU. Supports multiple clients. Auto-reconnects.
    # View at http://<roomba-ip>:8080/stream
    streamer = ExecuteProcess(
        cmd=[
            'ustreamer',
            '--device', '/dev/video0',
            '--host', '0.0.0.0',
            '--port', '8080',
            '--resolution', '640x480',
            '--format', 'MJPEG',
            '--desired-fps', '15',
        ],
        output='screen',
    )

    return LaunchDescription([streamer])
