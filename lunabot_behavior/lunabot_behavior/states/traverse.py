#!/usr/bin/env python3

from math import sqrt
from geometry_msgs.msg import PoseStamped
from lunabot_msgs.msg import Event
from rcl_interfaces.msg import ParameterType, ParameterValue, Parameter
import rclpy
from state import State, Events
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from rcl_interfaces.srv import SetParameters, GetParameters
import zones 

class TraverseToBerm(State): # TODO: ensure that we are actually
    def setup(self, manager: Node):
        self.goal_pub = manager.create_publisher(PoseStamped, "goal", 10)
        self.backwards_pub = manager.create_publisher(Bool, "traversal/backwards", 10)
        self.enabled_pub = manager.create_publisher(Bool, "traversal/enabled", 10)
        self.odom = None
        self.tolerance = 0.1
        self.goal = PoseStamped()
        self.goal.pose.position.x = (zones.berm_zone.v2.x + zones.berm_zone.v3.x) / 2
        self.goal.pose.position.y = (zones.berm_zone.v2.y + zones.berm_zone.v3.y) / 2
        self.goal.pose.position.z = (zones.berm_zone.v2.z + zones.berm_zone.v3.z) / 2
        self.goal.header.frame_id = "map"
        self.goal.header.stamp = manager.get_clock().now().to_msg()
        self.logger = manager.get_logger()

    def odom_cb(self, pose: PoseStamped):
        self.odom = pose

    def publish_everything(self):
        self.goal_pub.publish(self.goal)
        self.backwards_pub.publish(Bool(data = True))
        self.enabled_pub.publish(Bool(data = True))
    
    def start(self):
        self.publish_everything()

    def periodic(self) -> None | Events:
        self.publish_everything()
        return None

    def exit(self):
        self.logger.info("send disable");
        self.enabled_pub.publish(Bool(data = False))

class NoPath(State):
    def setup(self, manager: Node):
        self.waiting_for_planning = False
        self.waiting_for_future = False
        self.failed = True
        self.num_failed = 0
        self.failed_sub = manager.create_subscription(Bool, "nav/failed", self.failed_cb, 10)
        self.costmap_set_params_service = manager.create_client(SetParameters, "global_costmap/global_costmap/set_parameters")
        self.costmap_get_params_service = manager.create_client(GetParameters, "global_costmap/global_costmap/get_parameters")
        self.rtabmap_reset_service = manager.create_client(Empty, "rtabmap/rtabmap/reset")
        self.logger = manager.get_logger()
        self.manager = manager

    def failed_cb(self, value: Bool):
        self.waiting_for_planning = False
        self.failed = value.data

    def start(self):
        self.waiting_for_planning = False
        self.failed = True
        self.num_failed = 0

    def future_complete_cb(self, future: rclpy.Future):
        self.logger.info("future complete")
        self.waiting_for_future = False

    def radius_cb(self, future: rclpy.Future):
        get_response: GetParameters.Response = future.result()
        robot_radius: float = get_response.values[0].double_value
        self.logger.info(f"robot radius: {robot_radius}, new: {robot_radius * 2.0 / 3.0}")

        request = SetParameters.Request()
        request.parameters = [Parameter(name = "robot_radius", value = ParameterValue(double_value = robot_radius * 2.0 / 3.0, type = ParameterType.PARAMETER_DOUBLE))]
        self.costmap_set_params_service.call_async(request).add_done_callback(self.future_complete_cb)

    def periodic(self):
        if self.waiting_for_planning or self.waiting_for_future:
            self.logger.info(f"waiting {self.waiting_for_future} {self.waiting_for_planning}")
            return None
        elif not self.failed:
            self.logger.info("No path success")
            return Events.SUCCESS
        elif (self.num_failed == 0 or self.num_failed == 1):
            self.logger.info(f"failed: {self.num_failed}")
            get_request = GetParameters.Request(names = ["robot_radius"])
            self.costmap_get_params_service.call_async(get_request).add_done_callback(self.radius_cb)
            self.waiting_for_future = True
        elif (self.num_failed == 2):
            self.rtabmap_reset_service.call_async(Empty.Request()).add_done_callback(self.future_complete_cb)
            self.waiting_for_future = True
        elif (self.num_failed > 2):
            self.logger.info(f"failed final")
            return Events.FAIL

        self.num_failed += 1
        self.waiting_for_planning = True
