#!/usr/bin/env python3

import rospy

from lunabot_msgs.msg import RobotSensors
from std_msgs.msg import Int8, Int32

import time
import sys

class DepositionManager:
    '''
    Used to run deposition system
    '''

    def sensors_callback(self, msg: RobotSensors):
        self.robot_sensors = msg

    def __init__(self, deposition_publisher: rospy.Publisher = None):
        """
        If passed a publisher, then it is assumed a node is already running, and the publisher is shared.
        Else, initialize this node to run on its own.
        """

        if deposition_publisher is None:
            self.deposition_publisher = rospy.Publisher("/deposition", Int32, queue_size=1, latch=True)
            rospy.init_node('deposition_node')
        else:
            self.deposition_publisher = deposition_publisher

        self.robot_sensors = RobotSensors()

        rospy.Subscriber("/sensors", RobotSensors, self.sensors_callback)

        self.rate = rospy.Rate(10)  # 10hz

        self.DEPOSITION_SPEED = 2000 # In RPM, TODO pick a value

        self.WEIGHT_THRESHOLD = 1 # in kilograms, TODO choose value / verify WHEN LOAD CELLS EXIST

        self.DEPOSITION_TIME = 15.00

        self.is_sim = rospy.get_param("is_sim")

    def deposit(self):
        """
        Spin deposition until the load cell detects that the bin is empty (or up to a max time)
        """

        if (self.is_sim):
            rospy.loginfo("Deposition: would deposit")
            time.sleep(2)
            return True

        time.sleep(0.1)

        deposition_msg = Int32()
        deposition_msg.data = self.DEPOSITION_SPEED

        start_time = rospy.get_time()

        while True:
            self.deposition_publisher.publish(deposition_msg)
            
            if rospy.get_time() - start_time > self.DEPOSITION_TIME:
                break

            weight = self.robot_sensors.load_cell_weights[0] + self.robot_sensors.load_cell_weights[1]

            if weight < self.WEIGHT_THRESHOLD:
                break

            self.rate.sleep()

        deposition_msg.data = 0
        self.deposition_publisher.publish(deposition_msg)

    def no_sensor_deposit(self):
        """
        Spin the auger for a set amount of time
        """

        if (self.is_sim):
            rospy.loginfo("Deposition: would deposit")
            time.sleep(2)
            return True

        time.sleep(0.1)

        deposition_msg = Int32()
        deposition_msg.data = self.DEPOSITION_SPEED

        start_time = rospy.get_time()

        while True:
            self.deposition_publisher.publish(deposition_msg)
            
            if rospy.get_time() - start_time > self.DEPOSITION_TIME:
                break

            self.rate.sleep()

        deposition_msg.data = 0
        self.deposition_publisher.publish(deposition_msg)

        return True

    
if __name__ == "__main__":

    deposition = DepositionManager()

    if len(sys.argv) > 1 and (sys.argv[1] == "-n" or sys.argv[1] == "-no-sensor"):
        deposition.no_sensor_deposit()
    else:
        deposition.deposit()
