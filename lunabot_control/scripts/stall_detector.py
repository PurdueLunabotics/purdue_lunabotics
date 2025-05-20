#!/usr/bin/python3
import rospy

from lunabot_msgs.msg import RobotEffort, RobotSensors
from lunabot_msgs.msg import Bool

class StallDetector:
    def __init__(self):
        rospy.init_node("stall_detector")

        rospy.Subscriber("/effort", RobotEffort, self.effort_callback)
        rospy.Subscriber("/sensors", RobotSensors, self.sensors_callback)

        self.stall_publisher = rospy.Publisher("/stalled", Bool, queue_size=5, latch=True)
        self.stall_start_time = 0

        self.effort: RobotEffort = None

        self.counter = [0, 0, 0] # left_drive, right_drive, exc

    def effort_callback(self, msg: RobotEffort):
        self.effort = msg

    def sensors_callback(self, msg: RobotSensors):
        if rospy.Time().secs - self.stall_start_time > 5:
            # left drive
            if msg.drive_left_vel < 10 and self.effort.left_drive > 0:
                self.counter[0] += 1
            else:
                self.counter[0] = 0
            
            # right drive
            if msg.drive_right_vel < 10 and self.effort.right_drive > 0:
                self.counter[1] += 1
            else:
                self.counter[1] = 0

            # exc
            if msg.exc_vel < 10 and self.effort.excavate > 0:
                self.counter[2] += 1
            else:
                self.counter[2] = 0

            # loop through counters
            for count in self.counter:
                if count > 10:
                    self.stall_publisher.publish(True)
                    self.stall_start_time = rospy.Time().secs
                    break
            
if __name__ == "__main__":
    stall_detector = StallDetector()
    rospy.spin()