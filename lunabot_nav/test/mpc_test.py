#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped

class MPC_Test:
    def __init__(self):
        rospy.init_node('mpc_test')
        goal_pub = rospy.Publisher('lunabot_nav/')