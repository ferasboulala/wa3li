from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wa3li',
            executable='kobuki_node',
        ),
        Node(
            package='wa3li',
            executable='kinect_node',
        ),
        Node(
            package='wa3li',
            executable='nav_node',
        )
    ])
