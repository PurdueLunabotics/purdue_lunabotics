import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node, SetParameter, PushRosNamespace
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import xacro

def launch_setup(context):
    entity_name = LaunchConfiguration('entity_name').perform(context)
    nsConfig = LaunchConfiguration('ns')
    xacro_file = entity_name+'.xacro.urdf'
    package_description = "lunabot_description"
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "urdf", xacro_file)
    ns = nsConfig.perform(context)
    if (len(ns) > 0):
        ns = ns + "/"
    robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=entity_name,
        parameters=[{'frame_prefix': ns, 'use_sim_time': True, 'robot_description': Command(['xacro ', robot_desc_path, ' robot_name:=', entity_name])}],
        output="screen"
    )

    joint_state_publisher_node = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=entity_name,
        parameters=[{'frame_prefix': ns, 'use_sim_time': True, 'robot_description': Command(['xacro ', robot_desc_path, ' robot_name:=', entity_name])}],
        output="screen"
    )
    
    return [robot_state_publisher_node, joint_state_publisher_node]



def generate_launch_description():

    opfunc = OpaqueFunction(function = launch_setup)
    

    return LaunchDescription([
        DeclareLaunchArgument(
            'entity_name',
            default_value='dummy_bot'
        ),
        DeclareLaunchArgument(
            'ns',
            default_value=''
        ),
        opfunc
    ])
