from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='exercise41_pkg',
            executable='exercise43',
            output='screen',
            emulate_tty=True),
    ])