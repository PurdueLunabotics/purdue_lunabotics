import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = Command(
        [
            "xacro ",
            PathJoinSubstitution([
                FindPackageShare("lunabot_description"),
                "urdf",
                "dummy_bot.xacro"
            ])
        ]
    )
    
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # Controller parameters
    controller_params_file = PathJoinSubstitution([
        FindPackageShare("mining_arena_gazebo"),
        "config",
        "diff_drive_controller.yaml"
    ])

    # Controller manager
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_params_file],
        output="both",
    )
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    
    # Start Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("gazebo_ros"),
                "launch",
                "gazebo.launch.py"
            ])
        ])
    )
    
    # Spawn robot in Gazebo
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "dummy_bot"],
        output="screen"
    )

    # Load controllers with delays
    load_joint_state_broadcaster = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                output="screen"
            )
        ]
    )
    
    load_diff_drive_controller = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
                output="screen"
            )
        ]
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager,
        gazebo_launch,
        spawn_entity,
        load_joint_state_broadcaster,
        load_diff_drive_controller,
    ])
