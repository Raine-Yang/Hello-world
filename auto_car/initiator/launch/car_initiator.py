from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
            Node(
                package = 'camera',
                executable = 'camera_pub.py',
            ),
            Node(
                package = 'processor',
                executable = 'image_processor.py',
            ),
            Node(
                package = 'serial',
                executable = 'serial_controller.py'
            ),
    ])