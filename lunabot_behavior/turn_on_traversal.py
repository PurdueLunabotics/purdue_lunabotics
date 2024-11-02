#!/usr/bin/env python3

import rospy

from std_msgs.msg import Bool

rospy.init_node("turn_on_traversal_node")

traversal_publisher = rospy.Publisher("/behavior/traversal_enabled", Bool, queue_size=1, latch=True)

enabled_msg = Bool()
enabled_msg.data = True

print("Turning on mpc")

for i in range(10):
    traversal_publisher.publish(enabled_msg)
    rospy.sleep(0.05)