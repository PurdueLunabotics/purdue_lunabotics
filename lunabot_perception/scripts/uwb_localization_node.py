#!/usr/bin/env python3

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from lunabot_perception.algos.trilaterate import Trilaterate

from lunabot_msgs.msg import RobotState


class UWBLocalizationNode:
    def __init__(self):
        # Get rosparams for node positions
        self.uwb_pub_name = rospy.get_param("~uwb_pub_name", "/uwb_position")
        self.a1 = rospy.get_param("~uwb1pos", np.array([0, 0, 0]))  # uwb id:
        self.a2 = rospy.get_param(
            "~uwb2pos", np.array([0, 0, 0])
        )  # uwb_id: (uwb_base: MUST be 0,0,0)
        self.a3 = rospy.get_param("~uwb3pos", np.array([0, 0, 0]))  # uwb_id:

        # Init node
        rospy.init_node("uwb_localization_node", anonymous=False)
        # Start listener
        self.state_sub = rospy.Subscriber("state", RobotState, self.uwb_signal_sub)
        self.pos_pub = rospy.Publisher(self.uwb_pub_name, PoseStamped, queue_size=10)

        self.trilaterate = Trilaterate(self.a1, self.a2, self.a3)

        rospy.spin()

    def uwb_position_pub(self, pos):
        pos_vec = PoseStamped()
        pos_vec.header.stamp = rospy.Time.now()
        pos_vec.header.frame_id = "uwb_base"
        pos_vec.pose.position.x = pos[0]
        pos_vec.pose.position.y = pos[1]
        pos_vec.pose.position.z = pos[2]
        self.pos_pub.publish(pos_vec)

    def uwb_signal_sub(self, data):
        # Get data from vector
        D = data.uwb_dists
        D = np.array(D)
        pos = self.trilaterate(D)
        self.uwb_position_pub(pos)


if __name__ == "__main__":
    # Start node
    new_node = UWBLocalizationNode()
