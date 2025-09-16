from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
# from launch.condition import IFCondition
import os

def generate_launch_description():
    costmap_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        parameters=[{ 
          "autostart": True,
          "node_names":["costmap"]
        }],
        output='screen'
    )
    ld = LaunchDescription([
      TimerAction(period=10.0,
                  actions=[costmap_lifecycle])
    ])
    return ld