from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_controller',
            executable='obstacle_detector',
            name='obstacle_detector',
            parameters=[{'publish_debug_markers': True}],
        ),
        Node(
            package='my_robot_controller',
            executable='safety_filter',
            name='safety_filter',
        ),
        Node(
            package='my_robot_controller',
            executable='drive_forward',
            name='drive_forward',
            remappings=[('cmd_vel', 'cmd_vel_raw')],
        ),
    ])
