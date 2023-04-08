#!/usr/bin/env python3

import numpy as np
import rospy
import scipy.optimize as sci
from geometry_msgs.msg import Vector3


class node:
    # Function to minimize
    def S(self, v, d1, d2, d3):
        s = (
            (np.linalg.norm(v - self.a1) - d1) ** 2
            + (np.linalg.norm(v - self.a2) - d2) ** 2
            + (np.linalg.norm(v - self.a3) - d3) ** 2
        )
        return s

    def uwb_position_pub(self, pos):
        pub = rospy.Publisher(self.uwb_pub_name, Vector3, queue_size=10)
        pos_vec = Vector3()
        pos_vec.x = pub[0]
        pos_vec.y = pub[1]
        pos_vec.z = pub[2]
        pub.publish(pos_vec)

    def uwb_signal_sub(self, data):
        # Get data from vector
        d1 = data.x
        d2 = data.y
        d3 = data.z

        pos = sci.minimize(self.S, [0, 10, 0], args=(d1, d2, d3), method="Powell")
        self.uwb_position_pub(pos)

    def __init__(self):
        # Get rosparams for node positions
        self.uwb_pub_name = rospy.get_param("~uwb_pub_name")
        self.a1 = rospy.get_param("~node1pos")  # NOTE: PLACEHOLDER NAME
        self.a2 = rospy.get_param("~node2pos")  # NOTE: PLACEHOLDER NAME
        self.a3 = rospy.get_param("~node3pos")  # NOTE: PLACEHOLDER NAME

        # Init node
        rospy.init_node("UWB_localization_node", anonymous=False)

        # Start listener
        rospy.Subscriber("UWB_localization", Vector3, self.uwb_signal_sub)

        rospy.spin()


if __name__ == "__main__":
    # Start node
    new_node = node()
