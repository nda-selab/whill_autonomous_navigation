from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_yaml', description='OccupancyGrid map YAML used in Nav2'),
        DeclareLaunchArgument('waypoints_file', description='Separate output route YAML'),
        DeclareLaunchArgument('nav2_params_file', description='Nav2 parameter YAML file'),
        DeclareLaunchArgument('clearance', default_value='0.05'),
        Node(
            package='nav2_waypoint_tools', executable='waypoint_editor.py',
            name='waypoint_editor', output='screen', parameters=[{
                'map_yaml': LaunchConfiguration('map_yaml'),
                'waypoints_file': LaunchConfiguration('waypoints_file'),
                'nav2_params_file': LaunchConfiguration('nav2_params_file'),
                'clearance': LaunchConfiguration('clearance'),
            }],
        ),
    ])
