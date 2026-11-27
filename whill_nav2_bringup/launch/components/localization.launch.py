from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

def generate_launch_description():

    # =========================================================
    # SLAM framework (FAST-LIO2)
    # =========================================================
    fast_lio2 = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('fast_lio'),
                'launch',
                'mapping.launch.py',
            ])
        ),
        launch_arguments={
            'config_file': 'os2_64_whill.yaml',
            'rviz': 'false',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    # =========================================================
    # odom -> base_link
    # =========================================================
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

    # ==================================================
    # LiDAR Localization (map -> odom)
    # ==================================================
    lidar_localization = Node(
        package='lidar_localization_ros2',
        executable='lidar_localization_node',
        name='lidar_localization',
        output='screen',
        parameters=[
            LaunchConfiguration('localization_params_file'),
            {
                'map_path': LaunchConfiguration('prior_3d_map'),
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool
                ),
                'set_initial_pose': False,
                'global_frame_id': 'map',
                'odom_frame_id': 'odom',
                'base_frame_id': 'base_link',
                'enable_map_odom_tf': True,
                'use_odom_tf_prediction': True,
                'use_imu_preintegration': False,
                'use_imu': False,
                'use_odom': False,
                'use_twist_prediction': False,
                'use_continuous_time_deskew': False,
                'enable_timer_publishing': True,
                'pose_publish_frequency': 20.0,
                'publish_bridge_pose_when_lost': True,
            },
        ],
        remappings=[
            ('/cloud', '/cloud_registered'),
        ],
    )

    # ==================================================
    # LiDAR Localization Lifecycle
    # ==================================================
    localization_startup = Node(
        package='lidar_localization_ros2',
        executable='start_lifecycle_node.py',
        arguments=['lidar_localization'],
        output='screen',
    )

    # ==================================================
    # Launch
    # ==================================================
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
        ),
        DeclareLaunchArgument(
            'prior_3d_map',
            description='Absolute path to the prior 3D map',
        ),
        DeclareLaunchArgument(
            'localization_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('lidar_localization_ros2'),
                'param',
                'localization.yaml',
            ]),
        ),

        fast_lio2,
        odom_to_base_link,
        lidar_localization,
        localization_startup,
    ])
