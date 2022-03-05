#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
import math

class MPC_Test:
    def __init__(self, delta_t, robot_width, goal_x, goal_y):
        self.goal_x = goal_x
        self.goal_y = goal_y
        rospy.init_node('mpc_test')

        goal_pub = rospy.Publisher('lunabot_nav/goal', PoseStamped, 10)
        goal = PoseStamped()
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.header.frame_id = "base_link"
        goal.header.stamp = rospy.Time.now()
        goal_pub.publish(goal)
        self.path_arr = [goal, goal]

        self.odom_pub = rospy.Publisher('/odom', PoseStamped, 10)
        self.path_pub = rospy.Publisher('/path', PoseStamped, 10)
        rospy.Subscriber('lunabot_nav/cmd_vel', TwistStamped, self.update_odom)
        self.odom = Odometry()
        self.odom.pose.pose.position.x = 0
        self.odom.pose.pose.position.y = 0
        self.odom.pose.pose.orientation.x = 0
        self.odom.pose.pose.orientation.y = 0
        self.odom.pose.pose.orientation.z = 0
        self.odom.pose.pose.orientation.w = 1
        self.angle = 0
        self.delta_t = delta_t
        self.robot_width = robot_width
    
    def update_odom(self, vel):
        left_vel = vel.twist.linear.x - 0.5 * self.robot_width * vel.twist.angular.z
        right_vel = vel.twist.linear.x + 0.5 * self.robot_width * vel.twist.angular.z
        if(left_vel == right_vel):
            self.odom.pose.position.x += left_vel * math.cos(self.angle)
            self.odom.pose.position.y += left_vel * math.sin(self.angle)
        else:
            r = self.robot_width / 2 * (left_vel + right_vel) / (right_vel - left_vel)
            w = (right_vel - left_vel) / self.robot_width
            iccx = self.odom.pose.pose.position.x - r * math.sin(self.angle)
            iccy = self.odom.pose.pose.position.y + r * math.cos(self.angle)
            temp_x = self.odom.pose.pose.position.x
            temp_y = self.odom.pose.pose.position.y
            self.odom.pose.pose.position.x = math.cos(w * self.delta_t) * (temp_x - iccx) - math.sin(w * self.delta_t) * (temp_y - iccy) + iccx
            self.odom.pose.pose.position.y = math.sin(w * self.delta_t) * (temp_x - iccx) + math.cos(w * self.delta_t) * (temp_y - iccy) + iccy
            self.angle += w*self.delta_t
            self.odom.pose.pose.quaternion = quaternion_from_euler(0, 0, self.angle)
        path = Path()
        p1 = PoseStamped()
        p1.pose.position.x = self.odom.pose.pose.position.x
        p1.pose.position.y = self.odom.pose.pose.position.y
        p1.header.frame_id = "base_link"
        p1.header.stamp = rospy.Time.now()
        self.path_arr[0] = p1
        path.poses = self.path_arr
        path.header.frame_id = "base_link"
        path.header.stamp = rospy.Time.now()
        self.path_pub.publish(path)
        self.odom_pub.publish(self.odom)