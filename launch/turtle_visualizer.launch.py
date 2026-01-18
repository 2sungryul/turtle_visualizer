from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),
        Node(
            package='turtle_visualizer',
            executable='turtle_visualizer_node',
            name='turtle_visualizer',
            output='screen'
        )
    ])
