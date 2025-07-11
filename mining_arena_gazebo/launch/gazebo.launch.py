from socket import PACKET_HOST

from matplotlib.pyplot import get
from pytest import param
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, FindExecutable, Command
from launch_ros.substitutions import FindPackageShare
import os

import xacro

def generate_launch_description():
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
                'arena_nasa.world'

                # DUPAGE LOW RES WORLD DIR
                # 'low_resolution',
                # 'artemis',
                # 'artemis_arena.world'
            ])
        }.items()
    )

    robot_desc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lunabot_description'),
                'launch',
                'robot_description.launch.py'
            ])
        ])
    )

    robot_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'robot',
            '-x', '2.5', '-y', '1.75', '-z', '1.0',
            "--ros-args"
        ],
        output='screen'
    )

    ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[os.path.join(
            get_package_share_directory("mining_arena_gazebo"),
            "config",
            "ros2_control.yaml"
        )],
        output="screen"
    )

    # spawn_with_delay = TimerAction(
    #     period=10.0, # wait 10 sec before spawning to ensure that gazebo does in fact exist and the robot doesn't fall into the abyss
    #     actions=[robot_spawn_node]
    # )

    return LaunchDescription([
        gazebo_launch,
        robot_desc_launch,
        # spawn_with_delay,
        robot_spawn_node,
        ros2_control,
    ])