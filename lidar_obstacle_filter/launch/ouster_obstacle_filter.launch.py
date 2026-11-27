from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='lidar_obstacle_filter',
            executable='ouster_obstacle_filter.py',
            name='ouster_obstacle_filter',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('lidar_obstacle_filter'),
                    'config',
                    'obstacle_filter_params.yaml',
                ]),
                {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
            ],
        ),
    ])
