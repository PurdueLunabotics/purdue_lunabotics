#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8, Bool
from lunabot_msgs.msg import RobotSensors, RobotEffort

rospy.init_node("stop_node")

effort_publisher = rospy.Publisher("/effort", RobotEffort, queue_size=1)
cmd_vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
traversal_publisher = rospy.Publisher("/behavior/traversal_enabled", Bool, queue_size=1, latch=True)

effort = RobotEffort()
effort.excavate = 0
effort.lin_act = 0
effort.left_drive = 0
effort.right_drive = 0
effort.deposit = 0

cmd_vel = Twist()
cmd_vel.linear.x = 0
cmd_vel.angular.z = 0

disable_msg = Bool()
disable_msg.data = False

print("Stopping robot")

for i in range(10):
    traversal_publisher.publish(disable_msg)
    cmd_vel_publisher.publish(cmd_vel)
    effort_publisher.publish(effort)
    rospy.sleep(0.1)

