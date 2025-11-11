import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter, PushRosNamespace
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import xacro

def generate_launch_description():
    xacro_file = 'dummy_bot.xacro.urdf'

    
    package_description = "lunabot_description"
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "urdf", xacro_file)
    entity_name = "dummy_bot"

    robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=entity_name,
        parameters=[{'frame_prefix': '/', 'use_sim_time': True, 'robot_description': Command(['xacro ', robot_desc_path, ' robot_name:=', entity_name])}],
        output="screen"
    )

    joint_state_publisher_node = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=entity_name,
        parameters=[{'frame_prefix': '/', 'use_sim_time': True, 'robot_description': Command(['xacro ', robot_desc_path, ' robot_name:=', entity_name])}],
        output="screen"
    )
    
    xacro_file = 'mini_bot.xacro.urdf'
    package_description = "lunabot_description"
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "urdf", xacro_file)
    entity_name = "mini_bot"

    robot_state_publisher_node2 = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=entity_name,
        parameters=[{'frame_prefix': '/mini/', 'use_sim_time': True, 'robot_description': Command(['xacro ', robot_desc_path, ' robot_name:=', entity_name])}],
        output="screen"
    )

    joint_state_publisher_node2 = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=entity_name,
        parameters=[{'frame_prefix': '/mini/', 'use_sim_time': True, 'robot_description': Command(['xacro ', robot_desc_path, ' robot_name:=', entity_name])}],
        output="screen"
    )

    controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['velocity_controller'],
    )

    return LaunchDescription([
        robot_state_publisher_node, joint_state_publisher_node, controller,
        robot_state_publisher_node2, joint_state_publisher_node2
    ])
