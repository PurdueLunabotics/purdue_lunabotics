#!/usr/bin/env python3
import rospy

from sensor_msgs.msg import Joy
from lunabot_msgs.msg import RobotEffort, RobotErrors

from enum import Enum
import numpy as np

"""
.buttons   (1 is pressed, 0 not)
0 A
1 B
2 X
3 Y
4 LB
5 RB
6 back (view button)
7 start
8 power
9 button stick left
10 button stick right

.axes
0 left stick l/r     (left = 1.0, right = -1.0)
1 left stick u/d     (up = 1.0, down = -1.0)
2 Left Trigger       (Not pressed = 1.0, Fully depressed = -1.0)
3 right stick l/r    (left = 1.0, right = -1.0)
4 right stick u/d    (up = 1.0, down = -1.0)
5 Right Trigger      (Not pressed = 1.0, Fully depressed = -1.0)
6 dpad (arrows) l/r  (left = 1.0, right = -1.0)
7 dpad (arrows) u/d  (up = 1.0, down = -1.0)
"""
class Buttons(Enum):
    A = 0
    B = 1
    X = 2
    Y = 3
    LB = 4
    RB = 5
    BACK = 6
    START = 7
    POWER = 8
    L_STICK = 9
    R_STICK = 10

class Axes(Enum):
    L_STICK_HORIZONTAL = 0
    L_STICK_VERTICAL = 1
    LEFT_TRIGGER = 2
    R_STICK_HORIZONTAL = 3
    R_STICK_VERTICAL = 4
    RIGHT_TRIGGER = 5
    DPAD_HORIZONTAL = 6
    DPAD_VERTICAL = 7


class ManualController:
    """
    Manual Controller For Autonomy
    Current behavior is to not publish anything until the start button is pressed.
    When the start button is pressed, act as a complete stop.
    """

    def __init__(self):
        self.joy_subscriber = rospy.Subscriber("joy", Joy, self.joy_callback)
        self.effort_publisher = rospy.Publisher("effort", RobotEffort, queue_size=1, latch=True)
        self.effort_msg = RobotEffort()

        self.error_msg = RobotErrors()
        self.error_subscriber = rospy.Subscriber("errors", RobotErrors, self.error_callback)
        self.error_pub = rospy.Publisher("errors", RobotErrors, queue_size=1, latch=True)

    def error_callback(self, error_msg: RobotErrors):
        self.error_msg = error_msg

    def publish_manual_stop(self):
        self.error_msg.manual_stop = True
        self.error_pub.publish(self.error_msg)

    def unpublish_manual_stop(self):
        self.error_msg.manual_stop = False
        self.error_pub.publish(self.error_msg)

    def joy_callback(self, joy):
        # Start button: Stop the robot (Pause)
        if joy.buttons[Buttons.START.value] == 1:
            self.stop()
            self.publish_manual_stop()
            rospy.loginfo("Manual Control: Stopped")

        else:
            self.unpublish_manual_stop()

    def stop(self):
        self.effort_msg.left_drive = 0
        self.effort_msg.right_drive = 0
        self.effort_msg.excavate = 0
        self.effort_msg.lin_act = 0
        self.effort_msg.deposit = 0

        self._exc_latch_val = 0
        self._exc_latch = True

        self.effort_publisher.publish(self.effort_msg)


if __name__ == "__main__":
    rospy.init_node("manual_controller_node")

    man_ctrl = ManualController()

    rospy.spin()
