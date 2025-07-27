from posixpath import join
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, AppendEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

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

    ros_gz_sim = get_package_share_directory('ros_gz_sim')


    # add model directory to env variable so it knows where to look
    os.environ['GAZEBO_MODEL_PATH'] = f"$GAZEBO_MODEL_PATH:{get_package_share_directory('mining_arena_gazebo')}/models"

    world = PathJoinSubstitution([
                            FindPackageShare('mining_arena_gazebo'),
                            'worlds',

                            # OUR WORLD DIR
                            # 'arena_nasa.world'

                            # DUPAGE LOW RES WORLD DIR
                            'low_resolution',
                            'artemis',
                            'artemis_arena.world'
                        ])

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v4 '}.items()
    )

    robot_spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'dummy_bot',
            '-x', '2.5', '-y', '1.75', '-z', '1.0',
            '-Y', '1.570796327', # yaw
            "--ros-args"
        ],
        output='screen'
    )

    # ====================================
    # ROS2 CONTROL =======================

    # Joint state broadcaster
    # joint_state_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    #     output="screen"
    # )
    
    # # Diff drive controller
    # diff_drive_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
    #     output='screen'
    # )
    
    # Bridge ROS to Gazebo Topics
    bridge_params = os.path.join(
        get_package_share_directory('mining_arena_gazebo'),
        'params',
        'mining_arena_gazebo_bridge.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )
    
    start_gazebo_ros_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        output='screen',
    )
    ld = LaunchDescription([
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
        os.path.join(get_package_share_directory('mining_arena_gazebo'),
                     'models')),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=robot_spawn_node,
        #         on_exit=[joint_state_broadcaster_spawner],
        #     )
        # ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=joint_state_broadcaster_spawner,
        #         on_exit=[diff_drive_controller_spawner],
        #     )
        # ),

        gzserver_cmd,
        gzclient_cmd,
        robot_desc_launch,
        robot_spawn_node,
        start_gazebo_ros_bridge_cmd,
        start_gazebo_ros_image_bridge_cmd
    ])
    return ld