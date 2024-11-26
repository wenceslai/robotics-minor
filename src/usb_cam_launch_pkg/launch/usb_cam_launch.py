from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[{
                'video_device': '/dev/video1',  # Ensure this matches your video device
                'image_width': 640,
                'image_height': 480,
                'pixel_format': 'yuyv',
                'camera_name': 'usb_cam',
                'framerate': 30.0,
                'io_method': 'mmap',
 		'focus_auto': False,  # Disable auto focus
                'focus_absolute': 15,
            }]
        ),
    ])
