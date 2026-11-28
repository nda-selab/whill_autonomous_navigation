from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

def generate_launch_description():

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

                # Local prior map
                'enable_local_map_crop': True,
                'local_map_radius': 120.0,
                'local_map_update_distance': 2.0,
                'local_map_min_points': 100,
                'scan_max_range': 100.0,

                'enable_timer_publishing': True,
                'pose_publish_frequency': 20.0,
                'publish_bridge_pose_when_lost': True,
            },
        ],
        remappings=[
            ('/cloud', LaunchConfiguration('cloud_topic')),
        ],
    )

    localization_startup = Node(
        package='lidar_localization_ros2',
        executable='start_lifecycle_node.py',
        arguments=['lidar_localization'],
        output='screen',
    )

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
        DeclareLaunchArgument(
            'cloud_topic',
            default_value='/cloud_registered',
        ),

        lidar_localization,
        localization_startup,
    ])
