import ast

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, PushRosNamespace
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


CAMERAS = {
    "1": {"name": "d455_front", "serial_no": "141222070808"},
    "2": {"name": "d455_back", "serial_no": "349622073231"},
    "3": {"name": "d455_front", "serial_no": "339222072367"},
    "4": {"name": "d455_back", "serial_no": "939622074382"},
}


def _launch_cameras(context):
    robot_num = LaunchConfiguration("robot_num").perform(context)
    activated_cameras = ast.literal_eval(
        LaunchConfiguration("activated_cameras").perform(context)
    )
    if not activated_cameras:
        activated_cameras = ["3", "4"] if robot_num == "1" else ["1", "2"]
    profile = "848x480x30" if robot_num == "1" else "424x240x15"
    namespace = "mini" if robot_num == "1" else ""
    tf_prefix = f"{namespace}/" if namespace else ""

    camera_launches = []
    for camera_id in activated_cameras:
        camera_id = str(camera_id)
        camera = CAMERAS[camera_id]
        camera_profile = "640x480x15" if camera_id == "1" else profile
        camera_launches.append(
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("lunabot_perception"),
                            "launch",
                            "cameras_single.launch",
                        ]
                    )
                ),
                launch_arguments={
                    "sim": LaunchConfiguration("sim"),
                    "camera_name": camera["name"],
                    "serial_no": camera["serial_no"],
                    "profile": camera_profile,
                    "tf_prefix": tf_prefix,
                }.items(),
            )
        )

    return [GroupAction([PushRosNamespace(namespace), *camera_launches])] if namespace else camera_launches


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("sim", default_value="false"),
            DeclareLaunchArgument("lidar", default_value="false"),
            DeclareLaunchArgument("robot_num", default_value="0"),
            DeclareLaunchArgument(
                "activated_cameras",
                default_value="[]",
                description="List of camera IDs to launch.",
            ),
            DeclareLaunchArgument("pointcloud", default_value="true"),
            OpaqueFunction(function=_launch_cameras),
        ]
    )