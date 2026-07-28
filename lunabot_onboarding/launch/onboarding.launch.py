import launch
import launch_ros

def generate_launch_description():
    return launch.LaunchDescription([
            launch_ros.actions.Node(
                package='lunabot_onboarding',
                executable='effort_factory_node',
            ),
            launch_ros.actions.Node(
                package='lunabot_onboarding',
                executable='drive_controller_node',
            )
        ])
