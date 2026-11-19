from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_controller',
            executable='obstacle_detector',
            name='obstacle_detector',
            parameters=[{'publish_debug_markers': False}],
        ),
        Node(
            package='my_robot_controller',
            executable='debug_visualizer',
            name='debug_visualizer',
        ),
    ])
