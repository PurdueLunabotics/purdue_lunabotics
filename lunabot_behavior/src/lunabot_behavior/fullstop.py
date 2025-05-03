#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int32
from lunabot_msgs.msg import RobotSensors, RobotEffort

rospy.init_node("stop_node")

effort_publisher = rospy.Publisher("/effort", RobotEffort, queue_size=1)
cmd_vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
traversal_publisher = rospy.Publisher("/behavior/traversal_enabled", Bool, queue_size=1, latch=True)
excavate_publisher = rospy.Publisher("/excavate", Int32, queue_size=1)
deposit_publisher = rospy.Publisher("/deposition", Int32, queue_size=1)
left_publisher = rospy.Publisher("/left_drive", Int32, queue_size=1)
right_publisher = rospy.Publisher("/right_drive", Int32, queue_size=1)
linact_publisher = rospy.Publisher("/lin_act", Int32, queue_size=1)
led_publisher = rospy.Publisher("/led_color", Int32, queue_size=1, latch=True)


zero = Int32()
zero.data = 0

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
    excavate_publisher.publish(zero)
    linact_publisher.publish(zero)
    left_publisher.publish(zero)
    right_publisher.publish(zero)
    deposit_publisher.publish(zero)
    led_publisher.publish(zero)
    rospy.sleep(0.1)
