from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import OpaqueFunction, IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node, PushROSNamespace

cameras = [
    { "serial": "141222070808", "name": "d455_front" },
    { "serial": "349622073231", "name": "d455_back" },
    { "serial": "339222072367", "name": "d455_front" },
    { "serial": "939622074382", "name": "d455_back" },
]

def get_options(context: LaunchContext, name: str, main: str, mini: str):
    robot_num = int(LaunchConfiguration("robot_num").perform(context))
    option = LaunchConfiguration(name).perform(context)
    if len(option) > 0:
        return option
    elif robot_num == 1:
        return mini
    return main

def ns_trailing(ns: str):
    if len(ns) > 0:
        return f"{ns}/"
    return ""

def ns_leading(ns: str):
    if len(ns) > 0:
        return f"/{ns}"
    return ""

def launch_setup(context: LaunchContext):
    camera_list = get_options(context, "camera_list", "1,2", "3,4").split(",")

    profile = get_options(context, "profile", "424x240x15", "848x480x30")
    ns = get_options(context, "ns", "", "mini")

    sim = LaunchConfiguration("sim")
    pointcloud = bool(LaunchConfiguration("pointcloud").perform(context))

    camera_list_full = [cameras[int(camera) - 1] for camera in camera_list]

    launch_description = []

    for camera in camera_list_full:
        launch_description.append(IncludeLaunchDescription(
            launch_description_source=[FindPackageShare("lunabot_perception"), "/launch/cameras_single.launch"],
            launch_arguments={
                "camera_name": camera["name"],
                "serial_no": camera["serial"],
                "profile": profile,
                "tf_prefix": ns_trailing(ns),
                "sim": sim,
            }.items()
        ))

    if pointcloud:
        remappings = [(f"cloud{idx+1}", f"{ns_leading(ns)}/{camera["name"]}/points") for idx, camera in enumerate(cameras)]
        remappings.append(("combined_cloud", "depth_combined_cloud"))
        launch_description.append(
            GroupAction(
                actions=[
                    PushROSNamespace("pointcloud"),
                    Node(
                        package="rtabmap_util",
                        executable="point_cloud_aggregator",
                        name="point_cloud_aggregator",
                        respawn=True,
                        remappings=remappings,
                        parameters=[{
                            "count": 2,
                            "xyz_output": True,
                            "approx_sync": True,
                            "fixed_frame_id": f"{ns_trailing(ns)}base_link"
                        }],
                    )
                ]
            )
        )

    return launch_description

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_num", default_value="0"),
        DeclareLaunchArgument("profile", default_value=""),
        DeclareLaunchArgument("ns", default_value=""),
        DeclareLaunchArgument("camera_list", default_value=""),
        DeclareLaunchArgument("sim", default_value="false"),
        DeclareLaunchArgument("pointcloud", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
