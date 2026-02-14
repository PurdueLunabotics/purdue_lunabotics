#!/usr/bin/env python3

import threading
from enum import Enum

import numpy as np
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor, ParameterType
import rclpy
from geometry_msgs.msg import Point, Pose2D, PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from tf_transformations import euler_from_quaternion
from visualization_msgs.msg import Marker
from lunabot_msgs.msg import Event

from lunabot_control.pid_controller import PIDController


class States(Enum):
    MOVING_TO_ANGULAR_TARGET = 1
    MOVING_TO_LINEAR_TARGET = 2
    AT_DESTINATION = 4  # at final path point


class PointToPoint(Node):
    def __init__(self, **kwargs):
        super().__init__("point_to_point_node", **kwargs)
        rclpy.get_global_executor().add_node(self)
        # self.get_logger().info("init")

        self.declare_parameters("linear", [("p", 3.0, ParameterDescriptor(type = ParameterType.PARAMETER_DOUBLE)),
                                           ("i", 0.0, ParameterDescriptor(type = ParameterType.PARAMETER_DOUBLE)),
                                           ("d", 0.0, ParameterDescriptor(type = ParameterType.PARAMETER_DOUBLE)),
                                           ("max_speed", 0.3, ParameterDescriptor(type = ParameterType.PARAMETER_DOUBLE)),
                                           ("tolerance", 0.2, ParameterDescriptor(type = ParameterType.PARAMETER_DOUBLE))])

        self.declare_parameters("angular", [("p", 5.0, ParameterDescriptor(type = ParameterType.PARAMETER_DOUBLE)),
                                           ("i", 0.0, ParameterDescriptor(type = ParameterType.PARAMETER_DOUBLE)),
                                           ("d", 0.0, ParameterDescriptor(type = ParameterType.PARAMETER_DOUBLE)),
                                           ("max_speed", 60.0, ParameterDescriptor(type = ParameterType.PARAMETER_DOUBLE)),
                                           ("tolerance", 10.0, ParameterDescriptor(type = ParameterType.PARAMETER_DOUBLE))])

        self.declare_parameter("frequency", 60.0, ParameterDescriptor(type = ParameterType.PARAMETER_DOUBLE))

        self.LINEAR_P = self.get_parameter("linear.p").get_parameter_value().double_value
        self.LINEAR_I = self.get_parameter("linear.i").get_parameter_value().double_value
        self.LINEAR_D = self.get_parameter("linear.d").get_parameter_value().double_value
        self.LINEAR_TOLERANCE = self.get_parameter("linear.tolerance").get_parameter_value().double_value  # meters
        self.MAX_LINEAR_SPEED = self.get_parameter("linear.max_speed").get_parameter_value().double_value  # m/s
        self.linear_pid = PIDController(
            self.LINEAR_P,
            self.LINEAR_I,
            self.LINEAR_D,
            max_output=self.MAX_LINEAR_SPEED,
        )

        self.ANGULAR_P = self.get_parameter("angular.p").get_parameter_value().double_value
        self.ANGULAR_I = self.get_parameter("angular.i").get_parameter_value().double_value
        self.ANGULAR_D = self.get_parameter("angular.i").get_parameter_value().double_value
        self.ANGULAR_TOLERANCE_DEG = self.get_parameter("angular.tolerance").get_parameter_value().double_value
        self.ANGULAR_TOLERANCE_RAD = np.deg2rad(self.ANGULAR_TOLERANCE_DEG)
        self.MAX_ANGULAR_SPEED_DEG_PER_SEC = self.get_parameter("angular.max_speed").get_parameter_value().double_value
        self.MAX_ANGULAR_SPEED_RAD_PER_SEC = np.deg2rad(
            self.MAX_ANGULAR_SPEED_DEG_PER_SEC
        )
        self.angular_pid = PIDController(
            self.ANGULAR_P,
            self.ANGULAR_I,
            self.ANGULAR_D,
            max_output=self.MAX_ANGULAR_SPEED_RAD_PER_SEC,
        )

        self.robot_pose = [None, None, None]  # x, y, heading (rad)
        self.last_pose = [None, None, None]  # for velocity calculations

        self.FREQUENCY = self.get_parameter("frequency").get_parameter_value().double_value
        self.pid_dt = 1 / self.FREQUENCY
        self.prev_pid_time = 0
        self.odom_dt = 1 / self.FREQUENCY
        self.prev_odom_time = 0

        self.angular_vel = 0
        self.linear_vel = 0

        self.odom_velocity = [None, None]

        self.at_angle_target = True
        self.at_linear_target = True
        self.at_destination = True

        self.angle_error = 0
        self.linear_error = 0

        self.prev_linear_error = float("inf")

        self.target_pose_index = 0
        self.target_pose = [None, None, None]
        self.path = []

        self.state = States.AT_DESTINATION
        self.is_moving_backwards = False
        self.is_enabled = True

        self.print_debug_info: bool = False

        # PUBLISHERS ==================================================================================================
        cmd_vel_topic = "cmd_vel"
        self.cmd_vel_publisher = self.create_publisher(Twist, cmd_vel_topic, 10)

        # TODO: debugging
        self.angular_disparity_publisher = self.create_publisher(
            Float32, "ptp/angular_disparity", 10
        )
        self.linear_disparity_publisher = self.create_publisher(
            Float32, "ptp/linear_disparity", 10
        )
        self.heading_publisher = self.create_publisher(Float32, "ptp/heading", 10)
        self.angle_target_publisher = self.create_publisher(
            Float32, "ptp/angle_target", 10
        )
        self.pid_linear_publisher = self.create_publisher(Float32, "ptp/pid_linear", 10)
        self.pid_angular_publisher = self.create_publisher(
            Float32, "ptp/pid_angular", 10
        )

        self.path_segment_publisher = self.create_publisher(
            Marker, "ptp/current_target", 10
        )

        self.path_publisher = self.create_publisher(Marker, "ptp/line_path", 10)

        self.state_publisher = self.create_publisher(String, "ptp/robot_state", 10)

        self.target_publisher = self.create_publisher(Pose2D, "ptp/target_pose", 10)
        self.log_publisher = self.create_publisher(String, "ptp/log", 10)
        self.event_publisher = self.create_publisher(Event, "events", 10)
        # SUBSCRIBERS ==================================================================================================
        odom_topic = "odom"
        self.create_subscription(PoseStamped, odom_topic, self.__odom_callback, 1)

        path_topic = "global_path"
        self.create_subscription(Path, path_topic, self.__path_callback, 1)

        backwards_topic = "backwards"
        self.create_subscription(Bool, backwards_topic, self.__backwards_callback, 1)

        traversal_topic = "traversal_enabled"
        self.create_subscription(Bool, traversal_topic, self.__traversal_callback, 1)

        self.add_on_set_parameters_callback(self.__parameter_callback)

    # ==================================================================================================================
    # CALLBACKS
    # ==================================================================================================================

    def __backwards_callback(self, msg: Bool):
        self.is_moving_backwards = msg.data

    def __traversal_callback(self, msg: Bool):
        self.is_enabled = msg.data
        if not self.is_enabled:
            self.cmd_vel_publisher.publish(Twist())

    def __odom_callback(self, msg: PoseStamped):
        # self.get_logger().info("got Odom")
        # self.robot_velocity = [msg.twist.twist.linear, msg.twist.twist.angular]
        angles = euler_from_quaternion(
            [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ]
        )
        self.robot_pose = (
            msg.pose.position.x,
            msg.pose.position.y,
            (angles[2]) % (2 * np.pi) - np.pi
            if self.is_moving_backwards
            else angles[2],  # -pi to pi
        )

        self.odom_dt = (
            self.get_clock().now().seconds_nanoseconds()[0] - self.prev_odom_time
        )
        self.prev_odom_time = self.get_clock().now().seconds_nanoseconds()[0]

        if (
            self.robot_pose != [None, None, None]
            and self.last_pose != [None, None, None]
            and self.odom_dt != 0
        ):
            if self.odom_dt == 0 or self.odom_dt is None:
                self.odom_dt = 1 / self.FREQUENCY  # ensure no div by 0 errors

            # linear velocity
            self.odom_velocity[0] = (
                np.linalg.norm(
                    np.array(self.robot_pose[:2]) - np.array(self.last_pose[:2])
                )
                / self.odom_dt
            )
            # angular velocity
            self.odom_velocity[1] = (
                self.robot_pose[2] - self.last_pose[2]
            ) / self.odom_dt

        # update last position
        self.last_pose = self.robot_pose

    def __path_callback(self, msg: Path):
        if len(msg.poses) == 0:
            self.__visualize_line_path([])
            self.target_pose = [None, None, None]
            return

        self.path = []  # create list for storing all d* poses
        for pose in msg.poses:
            angles = euler_from_quaternion(
                [
                    pose.pose.orientation.x,
                    pose.pose.orientation.y,
                    pose.pose.orientation.z,
                    pose.pose.orientation.w,
                ]
            )

            # add all points to complex path
            self.path.append([pose.pose.position.x, pose.pose.position.y, angles[2]])

        # initialize target point
        self.target_pose_index = 0
        self.target_pose = self.path[self.target_pose_index]

    def __parameter_callback(self, params: list[rclpy.Parameter]):
        for param in params:
            if param.type_ != rclpy.Parameter.Type.DOUBLE:
                self.get_logger().error(f"Invalid parameter type for {param.name}")
                return SetParametersResult(successful = False, reason = f"Invalid parameter type for {param.name}")

            if param.name == "linear.p":
                self.linear_pid.kp = param.get_parameter_value().double_value
            elif param.name == "linear.d":
                self.linear_pid.kd = param.get_parameter_value().double_value
            elif param.name == "linear.i":
                self.linear_pid.ki = param.get_parameter_value().double_value
            elif param.name == "linear.max_speed":
                self.linear_pid.max_output = param.get_parameter_value().double_value
            elif param.name == "linear.tolerance":
                self.LINEAR_TOLERANCE = param.get_parameter_value().double_value
            elif param.name == "angular.p":
                self.angular_pid.kp = param.get_parameter_value().double_value
            elif param.name == "angular.d":
                self.angular_pid.kd = param.get_parameter_value().double_value
            elif param.name == "angular.i":
                self.angular_pid.ki = param.get_parameter_value().double_value
            elif param.name == "angular.max_speed":
                self.angular_pid.max_output = np.deg2rad(param.get_parameter_value().double_value)
            elif param.name == "angular.tolerance":
                self.ANGULAR_TOLERANCE_RAD = np.deg2rad(param.get_parameter_value().double_value)
            else:
                self.get_logger().error(f"Unknown parameter: {param.name}")
                return SetParametersResult(successful = False, reason = f"Unknown parameter: {param.name}")

        return SetParametersResult(successful = True)

    # ==================================================================================================================
    # STATE PROCESSING
    # ==================================================================================================================

    def __update_state(self, current_pose):
        """
        Calculates linear and angular error, and updates abstract robot state.

        Args:
            target (list-like): target 2D pose of format (x, y, theta)
            pose (list-like): current robot 2D pose in format (x, y, theta)
            path (list-like): sequence of target poses in the path, each of which are in format (x, y, theta)
        """

        # store x and y coords of pose in a location variable
        current_location = np.array(current_pose[:2])

        path = self.path

        # check if on final trajectory
        on_final_trajectory = len(path) == 0 or self.target_pose == path[-1]

        ## -------------------------------------------------
        ## CALCULATE WHETHER AT LINEAR TARGET -------
        ## -------------------------------------------------
        # calculate distance to target as error
        self.linear_error = np.linalg.norm(
            np.array(self.target_pose[:2]) - current_location
        )

        # check if robot linear position is within tolerance - if so, terminate linear motion
        self.at_linear_target = np.abs(self.linear_error) < self.LINEAR_TOLERANCE

        # ensure that error is negative if robot overshoots target
        if (
            np.abs(self.linear_error) - np.abs(self.prev_linear_error)
            >= self.LINEAR_TOLERANCE
        ):
            self.linear_error = self.linear_error * -1

        # update previous linear error after checking that the magnitude is decreasing
        self.prev_linear_error = self.linear_error

        ## -------------------------------------------------
        ## CALCULATE WHETHER AT ANGULAR TARGET -------
        ## -------------------------------------------------

        # calculate angle to target from x axis
        pose_target_angle = None
        if on_final_trajectory and self.at_linear_target:
            # set target to final path angle if reached linear destination
            pose_target_angle = self.target_pose[2]
        else:
            pose_target_angle = np.arctan2(  # calculate target angle [-pi,pi]
                self.target_pose[1] - current_pose[1],
                self.target_pose[0] - current_pose[0],
            )

        # subtract heading to find angle error
        self.angle_error = pose_target_angle - current_pose[2]

        # check if around-the-world distance is smaller than current different
        if np.abs(2 * np.pi - np.abs(self.angle_error)) < np.abs(self.angle_error):
            # normalize to make error reflect around-the-world
            self.angle_error = 2 * np.pi - np.abs(self.angle_error)
            # ensure that the direction is correct
            self.angle_error *= -1 if pose_target_angle - current_pose[2] > 0 else 1

        # check if robot heading is within tolerance - if so, terminate turning procedure
        self.at_angle_target = np.abs(self.angle_error) < self.ANGULAR_TOLERANCE_RAD

        ## -------------------------------------------------
        ## UPDATE STATE -------
        ## -------------------------------------------------

        if self.print_debug_info:
            self.get_logger().info(f"""
                   PTP ------------------------------- \n
                   At Linear Target: {self.at_linear_target} \n
                   At Angular Target: {self.at_angle_target} \n
                   On Final Trajectory: {on_final_trajectory} \n
                   -----------------------------------
                   """)
        # self.log_publisher.publish(f'''
        #            PTP ------------------------------- \n
        #            At Linear Target: {self.at_linear_target} \n
        #            At Angular Target: {self.at_angle_target} \n
        #            On Final Trajectory: {on_final_trajectory} \n
        #            -----------------------------------
        #            ''')
        if (not self.at_angle_target) and (not self.at_linear_target or on_final_trajectory):
            self.state = States.MOVING_TO_ANGULAR_TARGET
        elif not self.at_linear_target:
            self.state = States.MOVING_TO_LINEAR_TARGET
        else:  # move to angular target if angle target is not met
            if on_final_trajectory or len(self.path) <= self.target_pose_index:
                self.state = States.AT_DESTINATION  # update state if at destination
                self.event_publisher.publish(Event(data = Event.ARRIVED))
            else:
                # if at linear target and not on final trajectory, target point should update
                self.target_pose_index += 1
                self.target_pose = self.path[self.target_pose_index]
                self.__update_state(current_pose)

    # ==================================================================================================================
    # MOTION
    # ==================================================================================================================

    def __move_to_point(self):
        if self.state == States.MOVING_TO_ANGULAR_TARGET:
            # stop linear translation if angle error becomes too big
            self.linear_vel = 0

            self.angular_vel = -self.angular_pid.calculate(
                state=self.angle_error, dt=self.pid_dt, setpoint=0
            )

        elif self.state == States.MOVING_TO_LINEAR_TARGET:
            self.angular_vel = 0

            self.linear_vel = -self.linear_pid.calculate(
                state=self.linear_error, dt=self.pid_dt, setpoint=0
            )

        elif self.state == States.AT_DESTINATION:
            self.angular_vel = 0
            self.linear_vel = 0

    # ==================================================================================================================
    # VISUALIZATION
    # ==================================================================================================================

    def __visualize_path_to_target(self):
        marker = Marker()
        # Set the frame
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "target"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Set the position of the point
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        if (
            self.robot_pose[0] == None
            or self.robot_pose[1] == None
            or self.target_pose[0] == None
            or self.target_pose[1] == None
        ):
            return
        start_point = Point(
            x=float(self.robot_pose[0]), y=float(self.robot_pose[1]), z=0.0
        )
        end_point = Point(
            x=float(self.target_pose[0]), y=float(self.target_pose[1]), z=0.0
        )

        marker.points.append(start_point)
        marker.points.append(end_point)

        # Set line properties
        marker.scale.x = 0.01
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Alpha (transparency)

        self.path_segment_publisher.publish(marker)

    def __visualize_line_path(self, marker_list):
        marker = Marker()

        # Set the frame
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "target_paths"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        # Set the position of the point
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # set points to the list of marker points
        marker.points = marker_list

        # Set line properties
        marker.scale.x = 0.01
        marker.color.r = 1.0
        marker.color.g = 0.05
        marker.color.b = 1.0
        marker.color.a = 1.0  # Alpha (transparency)

        self.path_publisher.publish(marker)

    # ==================================================================================================================
    # NODE RUNNING
    # ==================================================================================================================

    def publish_telemetry(self):
        msg = String()
        msg.data = str(self.state)
        self.state_publisher.publish(msg)

        # publish velocity to cmd_vel
        vel = Twist()
        vel.linear.x = float(self.linear_vel)
        vel.angular.z = float(self.angular_vel)

        vel.linear.x *= -1 if self.is_moving_backwards else 1
        self.cmd_vel_publisher.publish(vel)

        try:
            # target pose publishing
            pose = Pose2D()
            pose.x = self.target_pose[0]
            pose.y = self.target_pose[1]
            pose.theta = self.target_pose[2]
            self.target_publisher.publish(pose)
        except:
            pass

        # visualize path to target
        if self.target_pose != [None, None]:
            self.__visualize_path_to_target()

    def run_node(self):
        rate = self.create_rate(self.FREQUENCY, self.get_clock())

        while rclpy.ok():
            # self.get_logger().info("loop")
            if self.is_enabled:
                pose = self.robot_pose

                # update difference in time
                self.pid_dt = (
                    self.get_clock().now().seconds_nanoseconds()[0] - self.prev_pid_time
                )
                self.prev_pid_time = (
                    self.get_clock().now().seconds_nanoseconds()[0]
                )  # update previous time

                if self.pid_dt == 0 or self.pid_dt is None:  # ensure no div by 0 errors
                    self.pid_dt = 1 / self.FREQUENCY

                if self.target_pose != [None, None, None] and pose != [
                    None,
                    None,
                    None,
                ]:
                    self.__update_state(pose)
                    self.__move_to_point()
                else:
                    self.linear_vel = 0
                    self.angular_vel = 0

                self.publish_telemetry()
            rate.sleep()


# ==================================================================================================================
# MAIN METHOD
# ==================================================================================================================
def spin_in_background():
    executor = rclpy.get_global_executor()
    try:
        executor.spin()
    except Exception:
        pass


def main():
    rclpy.init()
    t = threading.Thread(target=spin_in_background)
    t.start()
    point_to_point = PointToPoint()
    point_to_point.run_node()
