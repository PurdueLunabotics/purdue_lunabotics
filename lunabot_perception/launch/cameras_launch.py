from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushROSNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # args that can be set from the command line or a default will be used
        DeclareLaunchArgument('camera_name'),
        DeclareLaunchArgument('serial_no'),
        DeclareLaunchArgument('tf_prefix'),
        DeclareLaunchArgument('profile', default_value='640x480x15')

        Node(
            package='rtabmap_util'
            executable='point_cloud_xyz'
            name='point_cloud_xyz'
            respawn='true'
            remappings=[
                ('/depth/camera_info', '/aligned_depth_to_color/camera_info'),
                ('/depth/image', '/aligned_depth_to_color/image_raw'),
                ('/cloud', '/points'),
            ],
            parameters=[{
                'decimation': 4
                'voxel_size': 0.01
                'noise_filter_radius': 0.1
                'max_depth': 2.0
            }]
        )
    ])
