#!/usr/bin/env python3

import ast

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import GroupAction, IncludeLaunchDescription, PushRosNamespace
from launch.launch_description_sources import AnyLaunchDescriptionSource


CAMERAS = {
    "1": {"name": "d455_front", "serial_no": "141222070808"},
    "2": {"name": "d455_back", "serial_no": "349622073231"},
    "3": {"name": "d455_front", "serial_no": "339222072367"},
    "4": {"name": "d455_back", "serial_no": "939622074382"},
}


def generate_launch_description(robot_num=0, activated_cameras=None, sim=False, pointcloud=True):
    robot_num = str(robot_num)
    if activated_cameras is None:
        activated_cameras = ["3", "4"] if robot_num == "1" else ["1", "2"]
    elif isinstance(activated_cameras, str):
        activated_cameras = ast.literal_eval(activated_cameras)
    if not isinstance(activated_cameras, (list, tuple, set)):
        raise ValueError("activated_cameras must be a list, such as ['1', '2']")

    activated_cameras = [str(camera_id) for camera_id in activated_cameras]
    profile = "848x480x30" if robot_num == "1" else "424x240x15"
    namespace = "mini" if robot_num == "1" else ""
    tf_prefix = f"{namespace}/" if namespace else ""

    camera_launches = []
    for camera_id in activated_cameras:
        if camera_id not in CAMERAS:
            raise ValueError(f"Unknown camera ID: {camera_id}")

        camera = CAMERAS[camera_id]
        camera_profile = "640x480x15" if camera_id == "1" else profile
        camera_launches.append(
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    f"{get_package_share_directory('lunabot_perception')}/launch/cameras_single.launch"
                ),
                launch_arguments={
                    "sim": str(sim).lower(),
                    "camera_name": camera["name"],
                    "serial_no": camera["serial_no"],
                    "profile": camera_profile,
                    "tf_prefix": tf_prefix,
                    "pointcloud": str(pointcloud).lower(),
                }.items(),
            )
        )

    actions = [GroupAction([PushRosNamespace(namespace), *camera_launches])] if namespace else camera_launches
    return LaunchDescription(actions)


def main():
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    return launch_service.run()


if __name__ == "__main__":
    main()