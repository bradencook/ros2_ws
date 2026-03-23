from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='webcam',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',
                'image_size': [640, 480],
                'framerate': 30,
                'pixel_format': 'YUYV',
                'qos_overrides./image_raw.publisher.reliability': 'best_effort',
                'qos_overrides./image_raw.publisher.history': 'keep_last',
                'qos_overrides./image_raw.publisher.depth': 1,
                'qos_overrides./camera_info.publisher.reliability': 'best_effort',
                'qos_overrides./camera_info.publisher.history': 'keep_last',
                'qos_overrides./camera_info.publisher.depth': 1,
            }],
        ),
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            output='screen',
            parameters=[{
                'port': 8080,
                'address': '0.0.0.0',
            }],
        )
    ])