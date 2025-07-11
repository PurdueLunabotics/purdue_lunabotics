from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, FindExecutable, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
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

    # robot_desc_cmd = Command([
    #     FindExecutable(name='xacro'),
    #     ' ',
    #     PathJoinSubstitution([
    #         FindPackageShare('lunabot_description'),
    #         'urdf',
    #         'dummy_bot.xacro'
    #     ])
    # ])
    # robot_description = ParameterValue(
    #     robot_desc_cmd,
    #     value_type=str
    # )

    # robot_desc_config = xacro.process_file(os.path.join(
    #     get_package_share_directory("lunabot_description"),
    #     "urdf",
    #     "dummy_bot.xacro"
    # ))
    # robot_description = robot_desc_config.toxml()

    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     parameters=[{'robot_description': robot_description}],
    #     output='screen'
    # )

    # joint_state_publisher = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     output='screen'
    # )

    # ====================================
    # GAZEBO STUFF =======================

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

    # ====================================
    # ROS2 CONTROL =======================

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            # {'robot_description': robot_desc_config},
            os.path.join(
                get_package_share_directory("mining_arena_gazebo"),
                "config",
                "diff_drive_controller.yaml"
            )
        ],
        output="screen"
    )

    # Joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    
    # Diff drive controller
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
    )

    # diff_drive_controller = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['diff_drive_controller', '--param-file', os.path.join(
    #         get_package_share_directory("mining_arena_gazebo"),
    #         "config",
    #         "diff_drive_controller.yaml"
    #     )],
    #     output='screen',
    # )

    # diff_drive_controller = TimerAction(
    # period=5.0,  # wait 5 seconds
    #     actions=[
    #         Node(
    #             package='controller_manager',
    #             executable='spawner',
    #             arguments=['diff_drive_controller', '--param-file', os.path.join(
    #                 get_package_share_directory("mining_arena_gazebo"),
    #                 "config",
    #                 "diff_drive_controller.yaml"
    #             )],
    #             output='screen',
    #         )
    #     ]
    # )

    # spawn_with_delay = TimerAction(
    #     period=10.0, # wait 10 sec before spawning to ensure that gazebo does in fact exist and the robot doesn't fall into the abyss
    #     actions=[robot_spawn_node]
    # )

    return LaunchDescription([
        gazebo_launch,
        
        robot_desc_launch,
        # robot_state_publisher,
        # joint_state_publisher,

        robot_spawn_node,

        # ros2_control_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner
    ])