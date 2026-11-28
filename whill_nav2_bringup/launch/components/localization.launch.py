from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Localization launch directory
    launch_dir = PathJoinSubstitution([
        FindPackageShare('whill_nav2_bringup'),
        'launch', 'components', 'localization',
    ])

    # SLAM-based localization
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([launch_dir, 'slam.launch.py'])
        ),
        condition=LaunchConfigurationEquals('localization_mode', 'slam'),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    # EKF-based localization
    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([launch_dir, 'ekf.launch.py'])
        ),
        condition=LaunchConfigurationEquals('localization_mode', 'ekf'),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    # Global localization
    lidar_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([launch_dir, 'lidar_localization.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'localization_mode': LaunchConfiguration('localization_mode'),
            'prior_3d_map': LaunchConfiguration('prior_3d_map'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'localization_mode',
            default_value='slam',
            choices=['slam', 'ekf'],
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('prior_3d_map'),

        slam,
        ekf,
        lidar_localization,
    ])