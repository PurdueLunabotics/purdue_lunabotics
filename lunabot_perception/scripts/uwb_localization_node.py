#!/usr/bin/env python3

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped

from lunabot_msgs.msg import RobotSensors
from lunabot_perception.algos.trilaterate import Trilaterate


class UWBLocalizationNode:
    def __init__(self):
        # Get rosparams for node positions
        self.uwb_pub_name = rospy.get_param("~uwb_pub_name", "/uwb_position")
        self.a1 = np.array(rospy.get_param("~uwb1pos", [0, 1, 0]))  # uwb id: 1
        self.a2 = np.array(
            rospy.get_param("~uwb2pos", [0.95, 0, 0])
        )  # uwb_id: 2 (uwb_base: MUST be 0,0,0)
        self.a3 = np.array(rospy.get_param("~uwb3pos", [0, 0, 0]))  # uwb_id: 3

        # Init node
        rospy.init_node("uwb_localization_node", anonymous=False)
        # Start listener
        self.state_sub = rospy.Subscriber(
            "state", RobotSensors, self.uwb_signal_sub, queue_size=10
        )
        self.pos_pub = rospy.Publisher(self.uwb_pub_name, PoseStamped, queue_size=10)

        cfg = np.array([self.a1, self.a2, self.a3])
        print(cfg.shape)

        self.trilaterate = Trilaterate(cfg)

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
        pos = self.trilaterate.trilaterate(D)
        self.uwb_position_pub(pos)


if __name__ == "__main__":
    # Start node
    new_node = UWBLocalizationNode()
