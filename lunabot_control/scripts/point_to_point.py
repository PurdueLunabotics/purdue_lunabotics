#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, Point
from pid_controller import PIDController
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion
import numpy as np
from pid_controller import PIDController


class PointToPoint:
    def __init__(self):
        rospy.init_node('point_to_point_node')

        self.LINEAR_P = 1.
        self.LINEAR_I = 0
        self.LINEAR_D = 0
        self.LINEAR_TOLERANCE = 0.1  # meters
        self.MAX_LINEAR_SPEED = 1.  # m/s
        self.linear_pid = PIDController(
            self.LINEAR_P, self.LINEAR_I, self.LINEAR_D, max_output=self.MAX_LINEAR_SPEED)

        self.ANGULAR_P = 1.
        self.ANGULAR_I = 0
        self.ANGULAR_D = 0
        self.ANGULAR_TOLERANCE_DEG = 5
        self.ANGULAR_TOLERANCE_RAD = np.deg2rad(self.ANGULAR_TOLERANCE_DEG)
        self.MAX_ANGULAR_SPEED_DEG_PER_SEC = 360
        self.MAX_ANGULAR_SPEED_RAD_PER_SEC = np.deg2rad(
            self.MAX_ANGULAR_SPEED_DEG_PER_SEC)
        self.angular_pid = PIDController(
            self.ANGULAR_P, self.ANGULAR_I, self.ANGULAR_D, max_output=self.MAX_ANGULAR_SPEED_RAD_PER_SEC)

        self.robot_pose = [None, None, None]  # x, y, heading (rad)
        self.last_pose = [None, None, None]  # for velocity calculations

        self.FREQUENCY = 60
        self.pid_dt = 1 / self.FREQUENCY
        self.prev_pid_time = 0
        self.odom_dt = 1 / self.FREQUENCY
        self.prev_odom_time = 0

        self.angular_vel = 0
        self.linear_vel = 0

        self.target_point = [None, None]
        self.odom_velocity = [None, None]

        # PUBLISHERS ==================================================================================================
        cmd_vel_topic = rospy.get_param('/cmd_vel_topic', '/cmd_vel')
        self.cmd_vel_publisher = rospy.Publisher(
            cmd_vel_topic, Twist, queue_size=10)

        # TODO: debugging
        self.angular_disparity_publisher = rospy.Publisher(
            '/ptp/angular_disparity', Float32, queue_size=10)
        self.linear_disparity_publisher = rospy.Publisher(
            '/ptp/linear_disparity', Float32, queue_size=10)
        self.heading_publisher = rospy.Publisher(
            '/ptp/heading', Float32, queue_size=10)
        self.angle_target_publisher = rospy.Publisher(
            '/ptp/angle_target', Float32, queue_size=10)
        self.pid_linear_publisher = rospy.Publisher(
            '/ptp/pid_linear', Float32, queue_size=10)
        self.pid_angular_publisher = rospy.Publisher(
            '/ptp/pid_angular', Float32, queue_size=10)
        self.current_target_publisher = rospy.Publisher(
            '/ptp/current_target', Marker, queue_size=10)
        self.velocity_publisher = rospy.Publisher(
            '/ptp/velocity', Point, queue_size=10)

        # SUBSCRIBERS ==================================================================================================
        odom_topic = rospy.get_param('/odom_topic', '/odom')
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback)

    # ==================================================================================================================
    # CALLBACKS
    # ==================================================================================================================

    def odom_callback(self, msg: Odometry):
        # self.robot_velocity = [msg.twist.twist.linear, msg.twist.twist.angular]
        angles = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                       msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.robot_pose = (msg.pose.pose.position.x,
                           msg.pose.pose.position.y, angles[2])

        self.odom_dt = rospy.Time.now().to_sec() - self.prev_odom_time
        self.prev_odom_time = rospy.Time.now().to_sec()

        if (self.robot_pose != [None, None, None] and self.last_pose != [None, None, None] and self.odom_dt != 0):
            # linear velocity
            self.odom_velocity[0] = np.linalg.norm(
                np.array(self.robot_pose[:2]) - np.array(self.last_pose[:2])) / self.odom_dt
            # angular velocity
            self.odom_velocity[1] = (
                self.robot_pose[2] - self.last_pose[2]) / self.odom_dt

        # update last position
        self.last_pose = self.robot_pose

    def path_callback():
        pass

    # ==================================================================================================================
    # MOTION
    # ==================================================================================================================

    def turn_to_point(self, point, pose):
        # store heading for computation - resistant to changes in variable caused by odom callback during loop execution
        current_pose = pose
        # calculate angle to target from x axis
        pose_target_angle = np.arctan2(
            point[1] - self.robot_pose[1], point[0] - self.robot_pose[0])
        # subtract heading to find angle error
        angle_error = pose_target_angle - current_pose[2]
        if np.abs(2 * np.pi - angle_error) < np.abs(angle_error):
            # normalize to make error reflect around-the-world
            angle_error = 2 * np.pi - angle_error

        self.angular_vel = 0

        # check if robot heading is within tolerance - if so, terminate turning procedure
        at_angle_target = angle_error < self.ANGULAR_TOLERANCE_RAD

        if not at_angle_target:
            self.linear_vel = 0  # stop linear translation if angle error becomes too big

            self.angular_vel = -self.angular_pid.calculate(
                state=angle_error, dt=self.pid_dt, setpoint=0)

    def translate_to_point(self, point, pose):
        # store x and y coords of pose in a location variable
        current_pose = pose
        current_location = np.array(current_pose[:2])
        # calculate distance to target as error
        linear_error = np.linalg.norm(
            np.array(point[:2]) - current_location)

        self.linear_vel = 0

        # check if robot linear position is within tolerance - if so, terminate linear motion
        at_linear_target = linear_error < self.LINEAR_TOLERANCE

        if (not at_linear_target) and self.angular_vel == 0:  # only translate if stopped turning
            # update difference in time
            self.pid_dt = rospy.Time.now().to_sec() - self.prev_odom_time
            self.prev_odom_time = rospy.Time.now().to_sec()  # update previous time

            self.linear_vel = -self.linear_pid.calculate(
                state=linear_error, dt=self.pid_dt, setpoint=0)

        self.velocity_publisher.publish()

    def move_to_point(self, point):
        if point != [None, None] and self.robot_pose != [None, None, None]:
            self.target_point = point

            self.turn_to_point(self.target_point, self.robot_pose)
            self.translate_to_point(self.target_point, self.robot_pose)

        # publish velocity
        vel = Twist()
        vel.linear.x = self.linear_vel
        vel.angular.z = self.angular_vel
        self.cmd_vel_publisher.publish(vel)

        # publish target point
        if (self.target_point != [None, None]):
            marker = Marker()
            marker.header.frame_id = "odom"  # Set the frame, e.g., "map" or "base_link"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "target"
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            # Set the position of the point
            marker.pose.position.x = float(self.target_point[0])
            marker.pose.position.y = float(self.target_point[1])
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            # Set the scale of the sphere
            marker.scale.x = 0.2  # Diameter in x
            marker.scale.y = 0.2  # Diameter in y
            marker.scale.z = 0.2  # Diameter in z

            # Set the color of the sphere
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0  # Alpha (transparency)

            self.current_target_publisher.publish(marker)

    # ==================================================================================================================
    # NODE RUNNING
    # ==================================================================================================================

    def run_node(self):
        rate = rospy.Rate(self.FREQUENCY)

        while not rospy.is_shutdown():
            # update difference in time
            self.pid_dt = rospy.Time.now().to_sec() - self.prev_odom_time
            self.prev_pid_time = rospy.Time.now().to_sec()  # update previous time

            # feed in dummy target (2,2)
            self.move_to_point([1., 1., np.pi / 2])

            rate.sleep()


# ==================================================================================================================
# MAIN METHOD
# ==================================================================================================================

if __name__ == "__main__":
    point_to_point = PointToPoint()
    point_to_point.run_node()
