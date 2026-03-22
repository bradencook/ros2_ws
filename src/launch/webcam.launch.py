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
                'pixel_format': 'YUYV',  # stable format
                'camera_calibration_url': 'file:///home/ubuntu/.ros/camera_info/innomaker-u20cam-1080p-s1:_inno.yaml'
            }]
        )
    ])
