from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushROSNamespace
from launch_ros.substitutions import FindPackageShare
from launch.conditions import UnlessCondition


def generate_launch_description():
    return LaunchDescription([
        # args that can be set from the command line or a default will be used
        DeclareLaunchArgument('camera_name'),
        DeclareLaunchArgument('serial_no'),
        DeclareLaunchArgument('tf_prefix'),
        DeclareLaunchArgument('profile', default_value='640x480x15'),

        GroupAction(
            condition=UnlessCondition(LaunchConfiguration('sim')),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                            FindPackageShare('realsense2_camera'),
                            'launch',
                            'rs_launch.py'
                        ])
                    ]),
                    launch_arguments={
                        'camera_name': LaunchConfiguration('camera_name'),
                        'camera_namespace': '',
                        'base_frame_id': 'link',
                        'serial_no': LaunchConfiguration('serial_no'),
                        'initial_reset': 'true',
                        'pointcloud.enable': 'false',
                        'enable_sync': 'true',
                        'align_depth.enable': 'true',
                        'enable_depth': 'true',
                        'depth_module.depth_profile': LaunchConfiguration('profile'),
                        'enable_infra': 'true',
                        'enable_infra1': 'true',
                        'enable_infra2': 'true',
                        'depth_module.infra_profile': LaunchConfiguration('profile'),
                        'enable_color': 'true',
                        'rgb_camera.color_profile': LaunchConfiguration('profile'),
                        'decimation_filter.enable': 'true',
                        'publish_tf': 'true',
                        'tf_publish_rate': 0.0,
                        'tf_prefix': LaunchConfiguration('tf_prefix'),
                        'clip_distance': -1.0
                    }
                )
            ]
        ),

        PushROSNamespace(LaunchConfiguration('camera_name')),

        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('lunabot_perception'),
                'launch',
                'apriltag.launch'
            ]),
            launch_arguments={
                'camera_frame': [LaunchConfiguration('tf_prefix'), LaunchConfiguration('camera_name'), '_link'],
                'camera_topic': 'color/image_raw',
                'use_sim_time': LaunchConfiguration('sim')
            }
        ),
        
        Node(
            package='rtabmap_util',
            executable='point_cloud_xyz',
            name='point_cloud_xyz',
            respawn='true',
            remappings=[
                ('depth/camera_info', 'aligned_depth_to_color/camera_info'),
                ('depth/image', 'aligned_depth_to_color/image_raw'),
                ('cloud', 'points'),
            ],
            parameters=[{
                'decimation': 4,
                'voxel_size': 0.01,
                'noise_filter_radius': 0.1,
                'max_depth': 2.0,
            }]
        )
    ])
