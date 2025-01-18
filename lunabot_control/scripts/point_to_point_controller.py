#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, Point
from pid_controller import PIDController
from tf.transformations import euler_from_quaternion
import traceback
import numpy as np


class PointToPoint:
    def __init__(self):
        """
        Initializes the PointToPoint controller with the given PID values and tolerances.
        
        :param linear_kP: Proportional gain for linear velocity
        :param linear_kI: Integral gain for linear velocity
        :param linear_kD: Derivative gain for linear velocity
        :param angular_kP: Proportional gain for angular velocity
        :param angular_kI: Integral gain for angular velocity
        :param angular_kD: Derivative gain for angular velocity
        :param angular_tolerance: Tolerance for angular velocity
        :param pos_tolerance: Tolerance for position
        :param max_linear_vel: Maximum linear velocity
        :param max_angular_vel: Maximum angular velocity
        """

        rospy.init_node('point_to_point_node')

        self.LINEAR_P = 0.5
        self.LINEAR_I = 0
        self.LINEAR_D = 0

        self.ANGULAR_P = 0.5
        self.ANGULAR_I = 0
        self.ANGULAR_D = 0

        self.ANGULAR_TOLERANCE = 0.05 # radians
        self.POSITION_TOLERANCE = 0.1 # meters

        self.LINEAR_VELOCITY_TOLERANCE = 0.1 # m/s
        self.ANGULAR_VELOCITY_TOLERANCE = 0.1 # rad/s

        self.MAX_LINEAR_VELOCITY = 0.5
        self.MAX_ANGULAR_VELOCITY = 0.5

        self.linear_controller = PIDController(self.LINEAR_P, self.LINEAR_I, self.LINEAR_D)
        self.angular_controller = PIDController(self.ANGULAR_P, self.ANGULAR_I, self.ANGULAR_D)
        self.linear_controller.set_max_value(self.MAX_LINEAR_VELOCITY)
        self.angular_controller.set_max_value(self.MAX_ANGULAR_VELOCITY)
        self.linear_controller.set_tolerance(self.LINEAR_VELOCITY_TOLERANCE)
        self.angular_controller.set_tolerance(self.ANGULAR_VELOCITY_TOLERANCE)

        self.angular_velocity = 0
        self.linear_velocity = 0

        self.prev_dist_to_target = 0

        self.robot_pose = [float('inf'), float('inf'), float('inf')] # (x, y, theta)
        self.odom_velocity = [float('inf'), float('inf')] # [linear, angular]
        self.last_pos = [float('inf'), float('inf'), float('inf')]

        self.INIT_ROBOT_POSE = [float('inf'), float('inf'), float('inf')] # (x, y, theta)
        self.INIT_ODOM_VEL = [float('inf'), float('inf')] # [linear, angular]
        self.INIT_LAST_POS = [float('inf'), float('inf'), float('inf')]

        self.target = [float('inf'), float('inf')]
        self.INIT_TARGET = [float('inf'), float('inf')]
        self.path = [self.robot_pose[:2]]
        self.i = 0 # index of current target point

        self.FREQUENCY = 60
        self.dt = 1 / self.FREQUENCY
        self.prev_time = rospy.Time.now().to_sec()

        # SUBSCRIBERS =================================================================================================
        # path_topic = rospy.get_param('/nav/global_path_topic', '/nav/global_path')
        # rospy.Subscriber(path_topic, Path, self.read_path_callback)

        odom_topic = rospy.get_param('/odom_topic', '/odom')
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback)

        # PUBLISHERS ==================================================================================================
        cmd_vel_topic = rospy.get_param('/cmd_vel_topic', '/cmd_vel')
        self.cmd_vel_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

        # TODO: debugging
        self.angular_disparity_pub = rospy.Publisher('/ptp/angular_disparity', Float32, queue_size=10)
        self.linear_disparity_pub = rospy.Publisher('/ptp/linear_disparity', Float32, queue_size=10)
        self.heading_pub = rospy.Publisher('/ptp/heading', Float32, queue_size=10)
        self.angle_target_pub = rospy.Publisher('/ptp/angle_target', Float32, queue_size=10)
        self.pid_linear_pub = rospy.Publisher('/ptp/pid_linear', Float32, queue_size=10)
        self.pid_angular_pub = rospy.Publisher('/ptp/pid_angular', Float32, queue_size=10)
        self.current_target_pub = rospy.Publisher('/ptp/current_target', Point, queue_size=10)
        self.velocity_pub = rospy.Publisher('/ptp/velocity', Twist, queue_size=10)

    def read_path_callback(self, msg: Path):
        path = [self.robot_pose[:2]]
        for point in msg.poses:
            path.append(tuple((point.pose.position.x, point.pose.position.y)))

        if path != self.path: # only update path if it's different
            # print(path)
            self.path = path
            self.initialize_target_point(self.robot_pose)
            
    def odom_callback(self, msg: Odometry):
        # self.robot_velocity = [msg.twist.twist.linear, msg.twist.twist.angular]
        angles = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.robot_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, angles[2])
        
        if (self.dt == 0):
            self.dt = rospy.Time.now().to_sec() - self.prev_time
            self.prev_time = rospy.Time.now().to_sec()
            if (self.dt == 0):
                self.dt = 1 / self.FREQUENCY
                
        if (self.robot_pose != self.INIT_ROBOT_POSE and self.last_pos != self.INIT_LAST_POS):
            # linear velocity
            self.odom_velocity[0] = np.sqrt((self.robot_pose[0] - self.last_pos[0]) ** 2 + (self.robot_pose[1] - self.last_pos[1]) ** 2) / (self.dt)
            # angular velocity
            self.odom_velocity[1] = (self.robot_pose[2] - self.last_pos[2]) / self.dt

        # update last position
        self.last_pos = self.robot_pose

    def initialize_target_point(self, robot_pose: tuple) -> None:
        for i in range(len(self.path) - 1):
            if robot_pose[0] > self.path[i][0] and robot_pose[1] > self.path[i][1] and robot_pose[0] < self.path[i + 1][0] and robot_pose[1] < self.path[i + 1][1]:
                self.target = self.path[i + 1] # return next point if robot is in path
                self.i = i + 1
                return
            
        self.i = 0
        self.target = self.path[self.i]

    def follow_path(self):
        """
        Follows the path by turning to the target point and then moving towards it. Called periodically. 
        """
        self.target = [2, 0.25]
        
        if (self.target != self.INIT_TARGET and self.robot_pose != self.INIT_ROBOT_POSE):
            dist_to_target = np.sqrt((self.target[0] - self.robot_pose[0]) ** 2 + (self.target[1] - self.robot_pose[1]) ** 2)
            self.prev_dist_to_target = dist_to_target

            if (abs(self.prev_dist_to_target) < abs(dist_to_target)): # if distance to target is increasing, switch direction
                dist_to_target = -dist_to_target

            self.linear_disparity_pub.publish(dist_to_target) # TODO: debugging

            angle_target = np.arctan2(self.target[1] - self.robot_pose[1], self.target[0] - self.robot_pose[0])
            angular_disparity = angle_target - self.robot_pose[2]
            if np.abs(2 * np.pi - angular_disparity) < np.abs(angular_disparity):
                angular_disparity = 2 * np.pi - angular_disparity # normalize to make error reflect around-the-world

            self.heading_pub.publish(self.robot_pose[2]) # TODO: debugging
            self.angle_target_pub.publish(angle_target) # TODO: debugging
            self.angular_disparity_pub.publish(angular_disparity) # TODO: debugging

            if (self.dt == 0):
                self.dt = rospy.Time.now().to_sec() - self.prev_time
                self.prev_time = rospy.Time.now().to_sec()
                if (self.dt == 0):
                    self.dt = 1 / self.FREQUENCY

            angular_vel = -self.angular_controller.calculate(angular_disparity, self.dt, setpoint=0)
            linear_vel = -self.linear_controller.calculate(dist_to_target, self.dt, setpoint=0)

            at_angle_target = abs(angular_disparity) < self.ANGULAR_TOLERANCE and abs(angular_vel) < self.ANGULAR_VELOCITY_TOLERANCE
            at_linear_target = abs(dist_to_target) < self.POSITION_TOLERANCE and abs(linear_vel) < self.LINEAR_VELOCITY_TOLERANCE

            # turn to point
            if (not at_angle_target):
                self.linear_velocity = 0
                self.angular_velocity = angular_vel
            else: # if at angle target, translate to point
                self.angular_velocity = 0
                if (not at_linear_target):
                    self.linear_velocity = linear_vel
                else:
                    self.linear_velocity = 0

            if (at_angle_target and at_linear_target):
                if (self.i < len(self.path) - 1):
                    self.i += 1
                    self.target = self.path[self.i]

            self.pid_linear_pub.publish(self.linear_velocity) # TODO: debugging
            self.pid_angular_pub.publish(self.angular_velocity) # TODO: debugging

            rospy.loginfo(f"self.target: {self.target}, type: {type(self.target)}")
            rospy.loginfo(f"self.robot_pose: {self.robot_pose}, type: {type(self.robot_pose)}")

        else:
            self.linear_velocity = 0
            self.angular_velocity = 0

        self.publish_telemetry()

    def publish_telemetry(self):
        vel = Twist()
        vel.linear.x = self.clamp(self.linear_velocity, -self.MAX_LINEAR_VELOCITY, self.MAX_LINEAR_VELOCITY)
        vel.angular.z = self.clamp(self.angular_velocity, -self.MAX_ANGULAR_VELOCITY, self.MAX_ANGULAR_VELOCITY)
        self.cmd_vel_publisher.publish(vel)
        
        self.current_target_pub.publish(Point(self.target[0], self.target[1], 0)) # TODO: debugging

    def clamp(self, value, min_value, max_value) -> float:
        return max(min(value, max_value), min_value)

    def run_node(self):
        rate = rospy.Rate(self.FREQUENCY)

        while not rospy.is_shutdown():
            try:
                self.follow_path()
                self.publish_telemetry()
                self.dt = rospy.Time.now().to_sec() - self.prev_time
                self.prev_time = rospy.Time.now().to_sec()
            except Exception as e:
                rospy.logerr(f"ERROR OCCURRED HEEHAW: {e}")
                rospy.logerr(traceback.format_exc())
            
            rate.sleep()


if __name__ == '__main__':
    ptp = PointToPoint()
    ptp.run_node()