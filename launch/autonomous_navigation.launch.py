from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

def generate_launch_description():

    # =========================================================
    # System Status Announcer
    # =========================================================
    system_status_announcer = Node(
        package='whill_nav2_bringup',
        executable='system_status_announcer.py',
        name='system_status_announcer',
        output='screen',
        parameters=[
            LaunchConfiguration('announcer_params_file'),
            {
                'enable_speech': ParameterValue(
                    LaunchConfiguration('enable_voice_announcements'),
                    value_type=bool,
                ),
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'),
                    value_type=bool,
                ),
            },
        ],
    )

    # =========================================================
    # WHILL Driver
    # =========================================================
    whill_driver = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('whill_bringup'),
                'launch',
                'whill_launch.py',
            ])
        ),
    )

    # =========================================================
    # Static TF
    # =========================================================
    static_tf = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('whill_nav2_bringup'),
                'launch',
                'components',
                'static_tf.launch.py',
            ])
        )
    )

    # =========================================================
    # Ouster OS2-64
    # =========================================================
    os2_64 = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('whill_nav2_bringup'),
                'launch',
                'components',
                'os2_64.launch.py',
            ])
        ),
        launch_arguments={
            'os2_64_params_file': LaunchConfiguration('os2_64_params_file'),
        }.items(),
    )

    # =========================================================
    # Hokuyo URG
    # =========================================================
    urg = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('whill_nav2_bringup'),
                'launch',
                'components',
                'urg.launch.py',
            ])
        ),
        launch_arguments={
            'urg_params_file': LaunchConfiguration('urg_params_file'),
        }.items(),
    )

    # =========================================================
    # Obstacle Filters
    # =========================================================
    ouster_obstacle_filter = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('whill_nav2_bringup'),
                'launch',
                'components',
                'ouster_obstacle_filter.launch.py',
            ])
        )
    )

    urg_front_obstacle_filter = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('whill_nav2_bringup'),
                'launch',
                'components',
                'urg_front_obstacle_filter.launch.py',
            ])
        )
    )

    urg_side_obstacle_filter = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('whill_nav2_bringup'),
                'launch',
                'components',
                'urg_side_obstacle_filter.launch.py',
            ])
        )
    )

    # =========================================================
    # Localization
    # =========================================================
    localization = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('whill_nav2_bringup'),
                'launch',
                'components',
                'localization.launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'prior_3d_map': LaunchConfiguration('prior_3d_map'),
        }.items(),
    )

    # =========================================================
    # 速度監視
    # =========================================================
    cmd_vel_watchdog = Node(
        package='whill_nav2_bringup',
        executable='cmd_vel_watchdog.py',
        name='cmd_vel_watchdog',
        output='screen',
        parameters=[LaunchConfiguration('cmd_vel_watchdog_params_file')],
    )
    
    # =========================================================
    # Nav2
    # =========================================================
    nav2 = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('whill_nav2_bringup'),
                'launch',
                'components',
                'whill_nav2.launch.py',
            ])
        ),
        launch_arguments={
            'params_file': LaunchConfiguration('nav2_params_file'),
            'prior_2d_map': LaunchConfiguration('prior_2d_map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    # =========================================================
    # Visualization
    # =========================================================
    visualization = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('whill_nav2_bringup'),
                'launch',
                'components',
                'visualization.launch.py',
            ])
        ),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items(),
    )

    # =========================================================
    # Launch
    # =========================================================
    return LaunchDescription([

        DeclareLaunchArgument(
            'enable_voice_announcements',
            default_value='true',
            description='Enable voice status announcements',
        ),

        DeclareLaunchArgument(
            'announcer_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('whill_nav2_bringup'),
                'config',
                'system_status_announcer.yaml',
            ]),
            description='System status announcer parameter file',
        ),

        DeclareLaunchArgument(
            'os2_64_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('whill_nav2_bringup'),
                'config',
                'os2_64_params.yaml',
            ]),
            description='OS2-64 parameter YAML file',
        ),

        DeclareLaunchArgument(
            'urg_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('whill_nav2_bringup'),
                'config',
                'urg_params.yaml',
            ]),
            description='URG parameter YAML file',
        ),

        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('whill_nav2_bringup'),
                'config',
                'nav2_params.yaml',
            ]),
            description='Nav2 parameter YAML file',
        ),

        DeclareLaunchArgument(
            'cmd_vel_watchdog_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('whill_nav2_bringup'),
                'config',
                'cmd_vel_watchdog_params.yaml',
            ]),
            description='CmdVel Watchdog parameter YAML file',
        ),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz',
        ),

                DeclareLaunchArgument(
            'prior_2d_map',
            description='Absolute path to 2D occupancy grid map YAML',
        ),
        
        DeclareLaunchArgument(
            'prior_3d_map',
            description='Absolute path to prior 3D point cloud map',
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock',
        ),

        system_status_announcer,

        whill_driver,
        static_tf,
        os2_64,
        urg,

        TimerAction(period=5.0, actions=[
            localization,
        ]),

        TimerAction(period=10.0, actions=[
            ouster_obstacle_filter,
            urg_front_obstacle_filter,
            urg_side_obstacle_filter,
            cmd_vel_watchdog,
            nav2,
        ]),

        TimerAction(period=13.0, actions=[
            visualization,
        ]),
    ])
