#!/usr/bin/env python3

import sys
import os

import rospy
from std_msgs.msg import Int32

def log_internet(link_quality_publisher: rospy.Publisher, signal_level_publisher: rospy.Publisher):
    try:
        # run iwconfig in subshell
        output = os.popen("iwconfig 2>/dev/null | grep \"Link Quality\"").read()
        output = output.strip().replace("  "," ")

        # Expected Output: Link Quality=xx/100 Signal level=xx/100 Noise level=xx/100

        tokens = output.split(" ")

        if len(tokens) <= 1:
            raise Exception("Cannot parse iwconfig output")

        quality = tokens[1]  # "Quality=xx/100"

        quality = quality.split("=")[1]  # "xx/100"
        quality = int(quality.split("/")[0])  # xx

        signal_level = tokens[3]  # "Signal level=xx/100"
        signal_level = signal_level.split("=")[1]  # "xx/100"
        signal_level = int(signal_level.split("/")[0])  # xx

        link_quality_publisher.publish(quality)
        signal_level_publisher.publish(signal_level)
    except Exception as e:
        rospy.loginfo("Cannot log internet: " + str(e))
        link_quality_publisher.publish(-101)
        signal_level_publisher.publish(-101)


if __name__ == "__main__":
    rospy.init_node("internet_log")

    link_quality_publisher = rospy.Publisher("/internet/link_quality", Int32, queue_size=10)
    signal_level_publisher = rospy.Publisher("/internet/signal_level", Int32, queue_size=10)

    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        log_internet(link_quality_publisher, signal_level_publisher)
        rate.sleep()
