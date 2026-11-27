from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution([
        FindPackageShare('robot_system_monitor'),
        'config',
        'system_performance_monitor.yaml',
    ])
    return LaunchDescription([
        DeclareLaunchArgument('monitor_params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='robot_system_monitor',
            executable='system_performance_monitor.py',
            name='system_performance_monitor',
            output='screen',
            parameters=[
                LaunchConfiguration('monitor_params_file'),
                {'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool)},
            ],
        ),
    ])
