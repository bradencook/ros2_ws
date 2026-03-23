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
            }],
            arguments=[
                '--ros-args',
                '--qos-reliability', 'best_effort',
                '--qos-history', 'keep_last',
                '--qos-depth', '1',
            ],
        )
    ])