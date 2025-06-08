#!/usr/bin/python3
import rospy

from lunabot_msgs.msg import RobotEffort, RobotSensors
from std_msgs.msg import Bool

class StallDetector:
    def __init__(self):
        print("BAD")
        rospy.init_node("stall_detector")

        rospy.Subscriber("/effort", RobotEffort, self.effort_callback)
        rospy.Subscriber("/sensors", RobotSensors, self.sensors_callback)

        self.stall_publisher = rospy.Publisher("/stalled", Bool, queue_size=5, latch=True)
        self.stall_start_time = 0

        self.effort: RobotEffort = None

        self.counter = [0, 0, 0] # left_drive, right_drive, exc

        print("NOT BAD")

    def effort_callback(self, msg: RobotEffort):
        # print("EFFORT RECEIVED")
        self.effort = msg

    def sensors_callback(self, msg: RobotSensors):
        # print("SENSORS RECEIVED")
        if rospy.get_time() - self.stall_start_time > 5:
            # print("START TIME PASSED")

            # left drive
            if abs(msg.drive_left_vel) < 10 and abs(self.effort.left_drive) > 0:
                # print("L STALL????")
                self.counter[0] += 1
            else:
                self.counter[0] = 0
            
            # right drive
            if abs(msg.drive_right_vel) < 10 and abs(self.effort.right_drive) > 0:
                # print("R STALL????")
                self.counter[1] += 1
            else:
                self.counter[1] = 0

            # exc
            if abs(msg.exc_vel) < 10 and abs(self.effort.excavate) > 0:
                # print("Counter++")
                self.counter[2] += 1
            else:
                # print("EXC: Counter reset")
                self.counter[2] = 0

            # loop through counters
            for count in self.counter:
                if count > 10:
                    rospy.logerr("STALL DETECTED: " + str(count)) # 0 - left, 1 - right, 2 - excavation
                    self.stall_publisher.publish(True)
                    self.stall_start_time = rospy.get_time()
                    # self.counter = [0, 0, 0]
                    break
            
if __name__ == "__main__":
    stall_detector = StallDetector()
    rospy.spin()