from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

def generate_launch_description():

    # EKF
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

    # odom -> base_link
    odom_to_base_link = Node(
        package='whill_nav2_bringup',
        executable='odom_to_base_link.py',
        name='odom_to_base_link',
        output='screen',
        parameters=[{
            'input_topic': '/Odometry',
            'output_topic': '/odom_nav',
            'base_frame': 'base_link',
            'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'),
                value_type=bool,
            ),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
        ),

        ekf,
        odom_to_base_link,
    ])
