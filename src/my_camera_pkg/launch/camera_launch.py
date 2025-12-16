from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='my_camera_pkg',
                executable='camera_pub',
                name='publisher',
                output='screen'
            ),
            Node(
                package= 'my_camera_pkg',
                executable='camera_sub',
                name='subscriber',
                output = 'screen'
            ),
        ]
    )