from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='whill_nav2_bringup',
            executable='urg_obstacle_filter.py',
            name='urg_obstacle_filter',
            output='screen',
            parameters=[PathJoinSubstitution([
                FindPackageShare('whill_nav2_bringup'), 'config', 'urg_obstacle_filter.yaml',
            ])],
        ),
    ])
