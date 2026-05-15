#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from lunabot_msgs.msg import RobotEffort, RobotStall
from nav_msgs.msg import Path
from rcl_interfaces.msg import ParameterType, ParameterValue, Parameter
import rclpy
from rclpy.time import Duration
from lunabot_behavior.state import State, Events
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from rcl_interfaces.srv import SetParameters, GetParameters
import math

class Traverse(State):
    def __init__(self, goal: PoseStamped | None, backwards: bool):
        self.goal = goal
        self.backwards = backwards

    def set_goal(self, goal: PoseStamped):
        self.goal = goal

    def set_backwards(self, backwards: bool):
        self.backwards = backwards

    def setup(self, manager: Node):
        self.goal_pub = manager.create_publisher(PoseStamped, "goal", 10)
        self.backwards_pub = manager.create_publisher(Bool, "traversal/backwards", 10)
        self.enabled_pub = manager.create_publisher(Bool, "traversal/enabled", 10)
        self.odom_sub = manager.create_subscription(PoseStamped, "position", self.odom_cb, 10)
        self.path_sub = manager.create_subscription(Path, "nav_path", self.path_cb, 10)
        self.odom = None
        self.last_pose: None | PoseStamped = None
        self.tolerance = 0.2
        self.logger = manager.get_logger()
        self.manager = manager

    def path_cb(self, path: Path):
        self.last_pose = path.poses[-1] # type: ignore

    def odom_cb(self, pose: PoseStamped):
        self.odom = pose

    def publish_everything(self):
        if self.goal is not None:
            self.goal_pub.publish(self.goal)
        self.backwards_pub.publish(Bool(data = self.backwards))
        self.enabled_pub.publish(Bool(data = True))
    
    def start(self):
        self.publish_everything()
        self.start_time = self.manager.get_clock().now()

    def periodic(self) -> None | Events:
        if self.odom is None or self.last_pose is None:
            self.logger.warn("[Traverse] no odom or path")
            return None
        self.publish_everything()
        dist = math.sqrt((self.odom.pose.position.x - self.last_pose.pose.position.x) ** 2 + (self.odom.pose.position.y - self.last_pose.pose.position.y) ** 2)
        self.logger.debug(f"[Traverse]: distance {dist}")
        elapsed = self.manager.get_clock().now() - self.start_time
        if dist < self.tolerance and elapsed > Duration(seconds=10):
            return Events.SUCCESS
        return None

    def exit(self, event):
        self.logger.info("[Traverse]: send disable")
        self.enabled_pub.publish(Bool(data = False))

num_failed = 0

class NoPath(State):
    def setup(self, manager: Node):
        self.waiting_for_planning = False
        self.waiting_for_reset = False
        self.waiting_for_set_radius = False

        self.failed_sub = manager.create_subscription(Bool, "nav/failed", self.failed_cb, 10)
        self.costmap_set_params_service = manager.create_client(SetParameters, "global_costmap/global_costmap/set_parameters")
        self.costmap_get_params_service = manager.create_client(GetParameters, "global_costmap/global_costmap/get_parameters")
        self.rtabmap_reset_service = manager.create_client(Empty, "rtabmap/rtabmap/reset")

        self.failed = True
        self.initial_radius: None | float = None

        self.logger = manager.get_logger()
        self.manager = manager

        # self.logger.info("[No Path]: Waiting for costmap service")
        self.costmap_get_params_service.wait_for_service()
        self.costmap_set_params_service.wait_for_service()
        # self.logger.info("[No Path]: Waiting for rtabmap service")
        self.rtabmap_reset_service.wait_for_service()

        get_request = GetParameters.Request(names = ["robot_radius"])
        fut = self.costmap_get_params_service.call_async(get_request)
        # self.logger.info(f"[No Path]: fut: {fut.result()}")
        fut.add_done_callback(self.radius_cb)

    def failed_cb(self, value: Bool):
        self.waiting_for_planning = False
        self.failed = value.data

    def start(self):
        self.waiting_for_planning = False
        self.failed = True

    def reset_cb(self, future: rclpy.Future):
        self.waiting_for_reset = False

    def set_radius_cb(self, future: rclpy.Future):
        self.waiting_for_set_radius = False

    def radius_cb(self, future: rclpy.Future):
        get_response: GetParameters.Response = future.result()
        # self.logger.info(f"[No Path]: got radius: {get_response.values[0].double_value}")
        self.initial_radius = get_response.values[0].double_value

    def periodic(self):
        global num_failed

        if self.waiting_for_planning or self.waiting_for_set_radius or self.waiting_for_reset or self.initial_radius == None:
            return None
        elif not self.failed:
            return Events.SUCCESS
        elif (num_failed == 0 or num_failed == 1):
            self.waiting_for_set_radius = True
            request = SetParameters.Request()
            request.parameters = [Parameter(name = "robot_radius", value = ParameterValue(double_value = self.initial_radius * math.pow(2.0 / 3.0, num_failed + 1), type = ParameterType.PARAMETER_DOUBLE))]
            self.costmap_set_params_service.call_async(request).add_done_callback(self.set_radius_cb)
        elif (num_failed == 2):
            self.waiting_for_set_radius = True
            request = SetParameters.Request()
            request.parameters = [Parameter(name = "robot_radius", value = ParameterValue(double_value = self.initial_radius, type = ParameterType.PARAMETER_DOUBLE))]
            self.costmap_set_params_service.call_async(request).add_done_callback(self.set_radius_cb)

            self.waiting_for_reset = True
            self.rtabmap_reset_service.call_async(Empty.Request()).add_done_callback(self.reset_cb)
        elif (num_failed > 2):
            self.logger.info(f"[No Path]: failed final")
            num_failed = 0
            return Events.FAIL

        num_failed += 1
        self.waiting_for_planning = True

class Stall(State):
    def setup(self, manager: Node):
        self.logger = manager.get_logger()
        self.manager = manager
        self.effort_pub = manager.create_publisher(RobotEffort, "effort", 10)
        self.stalled_sub = manager.create_subscription(RobotStall, "stalled", self.stalled_cb, 1)
        if not manager.has_parameter("stall.wait_duration_seconds"):
            manager.declare_parameter("stall.wait_duration_seconds", 2.0)
        self.stalled = RobotStall()

    def stalled_cb(self, stalled: RobotStall):
        self.stalled = stalled

    def start(self):
        effort = RobotEffort()
        effort.should_reset = True
        self.effort_pub.publish(effort)
        self.start_time = self.manager.get_clock().now()

    def periodic(self) -> None | Events:
        duration: Duration = self.manager.get_clock().now() - self.start_time
        if duration.nanoseconds / 1e9 > self.manager.get_parameter("stall.wait_duration_seconds").get_parameter_value().double_value:
            return Events.SUCCESS
        return None
