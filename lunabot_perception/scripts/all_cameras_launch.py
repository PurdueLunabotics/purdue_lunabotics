#!/usr/bin/env python3

import ast
import threading
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import GroupAction, IncludeLaunchDescription, PushRosNamespace
from launch.launch_description_sources import AnyLaunchDescriptionSource
from rclpy.node import Node


CAMERAS = {
    "1": {"name": "d455_front", "serial_no": "141222070808"},
    "2": {"name": "d455_back", "serial_no": "349622073231"},
    "3": {"name": "d455_front", "serial_no": "339222072367"},
    "4": {"name": "d455_back", "serial_no": "939622074382"},
}


class AllCamerasNode(Node):
    def __init__(self):
        super().__init__("all_cameras")
        self.declare_parameter("sim", False)
        self.declare_parameter("lidar", False)
        self.declare_parameter("robot_num", 0)
        self.declare_parameter("activated_cameras", "[]")
        self.declare_parameter("pointcloud", True)

        robot_num = str(self.get_parameter("robot_num").value)
        activated_cameras = self.get_parameter("activated_cameras").value
        if isinstance(activated_cameras, str):
            activated_cameras = ast.literal_eval(activated_cameras)
        if not isinstance(activated_cameras, list):
            raise ValueError("activated_cameras must be a list, such as ['1', '2']")

        if not activated_cameras:
            activated_cameras = ["3", "4"] if robot_num == "1" else ["1", "2"]
        profile = "848x480x30" if robot_num == "1" else "424x240x15"
        namespace = "mini" if robot_num == "1" else ""
        tf_prefix = f"{namespace}/" if namespace else ""

        camera_launches = []
        for camera_id in activated_cameras:
            camera_id = str(camera_id)
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
                        "sim": str(self.get_parameter("sim").value).lower(),
                        "camera_name": camera["name"],
                        "serial_no": camera["serial_no"],
                        "profile": camera_profile,
                        "tf_prefix": tf_prefix,
                    }.items(),
                )
            )

        actions = [GroupAction([PushRosNamespace(namespace), *camera_launches])] if namespace else camera_launches
        self.launch_service = LaunchService()
        self.launch_service.include_launch_description(LaunchDescription(actions))
        self.launch_thread = threading.Thread(target=self.launch_service.run, daemon=True)
        self.launch_thread.start()


def main(args=None):
    rclpy.init(args=args)
    node = AllCamerasNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.launch_service.shutdown()
        node.launch_thread.join()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()