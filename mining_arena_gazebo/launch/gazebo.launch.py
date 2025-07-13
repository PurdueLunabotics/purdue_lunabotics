from posixpath import join
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, FindExecutable, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os

import xacro

def generate_launch_description():
    robot_desc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lunabot_description'),
                'launch',
                'robot_description.launch.py'
            ])
        ])
    )

    # ====================================
    # GAZEBO STUFF =======================

    os.environ['GAZEBO_MODEL_PATH'] = f"$GAZEBO_MODEL_PATH:{get_package_share_directory('mining_arena_gazebo')}/models/"

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                FindPackageShare('mining_arena_gazebo'),
                'worlds',

                # OUR WORLD DIR
                # 'arena_nasa.world'

                # DUPAGE LOW RES WORLD DIR
                'low_resolution',
                'artemis',
                'artemis_arena.world'
            ])
        }.items()
    )

    robot_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'dummy_bot',
            '-x', '2.5', '-y', '1.75', '-z', '1.0',
            '-Y', '1.570796327', # yaw
            "--ros-args"
        ],
        output='screen'
    )

    # ====================================
    # ROS2 CONTROL =======================

    # Joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )
    
    # Diff drive controller
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
        output='screen'
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=robot_spawn_node,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[diff_drive_controller_spawner],
            )
        ),

        gazebo_launch,
        robot_desc_launch,
        robot_spawn_node
    ])