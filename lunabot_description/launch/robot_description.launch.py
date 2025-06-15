import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
import xacro

def generate_launch_description():
    # path = os.path.join(get_package_share_directory('lunabot_description'))
    # xacro_file = os.path.join(path, 'urdf', 'dummy_bot.xacro')
    # doc = xacro.parse(open(xacro_file))
    # xacro.process_doc(doc)
    # robot_description = doc.toxml()

    robot_description = Command([
        FindExecutable(name='xacro'),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('lunabot_description'),
            'urdf',
            'dummy_bot.xacro'
        ])
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher
    ])
