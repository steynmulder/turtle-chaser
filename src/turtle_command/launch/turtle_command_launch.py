from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtle_command',
            executable='turtle_runner',
            name='turtle_runner',
            output='screen'
        ),
        Node(
            package='turtle_command',
            executable='turtle_chaser',
            name='turtle_chaser',
            output='screen'
        )
    ])