from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

def generate_launch_description():

    # Wheel Odometry
    wheel_odometry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('whill_wheel_odometry'),
                'launch',
                'wheel_odometry.launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('wheel_odom_params_file'),
        }.items(),
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            LaunchConfiguration('ekf_params_file'),
            {'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'),
                value_type=bool,
            )},
        ],
        remappings=[
            ('/odometry/filtered', '/odom_nav'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
        ),
        DeclareLaunchArgument(
            'wheel_odom_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('whill_wheel_odometry'),
                'config',
                'wheel_odometry.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'ekf_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('whill_nav2_bringup'),
                'config',
                'ekf_params.yaml',
            ]),
        ),

        wheel_odometry,
        ekf,
    ])
