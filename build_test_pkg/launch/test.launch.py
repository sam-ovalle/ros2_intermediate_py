from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='build_test_pkg',
            executable='script1_exe',
            output='screen'),
    ])