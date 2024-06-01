from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera',
            parameters=[{
                'video_device': '/dev/video0',  # Adjust if necessary
                'image_size': [640, 480],
                'pixel_format': 'YUYV',
                'camera_info_url': '',
                'io_method': 'mmap',
                'output_encoding': 'rgb8'
            }],
            output='screen'
        ),
    ])
