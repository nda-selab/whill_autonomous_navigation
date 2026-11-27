from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cmd_vel_watchdog',
            executable='cmd_vel_watchdog.py',
            name='cmd_vel_watchdog',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('cmd_vel_watchdog'),
                    'config',
                    'cmd_vel_watchdog_params.yaml',
                ]),
            ],
        ),
    ])
