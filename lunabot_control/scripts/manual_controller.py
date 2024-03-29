#!/usr/bin/env python3
import numpy as np
import rospy
from sensor_msgs.msg import Joy

"""
.buttons
0 A
1 B
2 X
3 Y
4 LB
5 RB
6 back
7 start
8 power
9 button stick left
10 button stick right

.axes
0 l/r stick left
1 u/d stick left
2 LT
3 l/r stick right
"""
from lunabot_msgs.msg import RobotEffort


def constrain(joy_in):
    return np.int8(np.clip(joy_in, -1, 1) * 127)


class ManualController:
    def __init__(self):
        self._joy_sub = rospy.Subscriber("joy", Joy, self.joy_cb)
        self._effort_pub_ = rospy.Publisher("effort", RobotEffort, queue_size=1)
        self._effort_msg = RobotEffort()
        self._driving_mode = True

        debug_str = "DRIVE" if self._driving_mode else "EXCAVATE"
        rospy.loginfo(f"MODE: {debug_str}")

        self._autonomy = False
        self._exc_latch_val = 0
        self._exc_latch = True
        self.stop()

    def joy_cb(self, joy):
        if joy.buttons[2]:  # 'X' button, switch between excavation mode and drive mode
            self._driving_mode = not self._driving_mode
            debug_str = "DRIVE" if self._driving_mode else "EXCAVATE"
            rospy.loginfo(f"MODE: {debug_str}")

        if joy.buttons[0]:  # 'A' button
            self._autonomy = not self._autonomy
            debug_str = "AUTONOMY" if self._autonomy else "MANUAL"
            rospy.loginfo(f"MODE: {debug_str}")
        if joy.buttons[1]:  # 'B' button
            self.stop()
            rospy.loginfo("STOPPED")
        else:
            effort_msg = RobotEffort()

            # Joysticks
            effort_msg.left_drive = 0
            effort_msg.right_drive = 0

            effort_msg.lead_screw = 0
            effort_msg.excavate = 0

            if self._driving_mode:
                # Joysticks
                effort_msg.left_drive = constrain(joy.axes[1])
                effort_msg.right_drive = constrain(joy.axes[4])
            else:
                if joy.buttons[3] == 1:  # Y button for latch
                    self._exc_latch_val = constrain(joy.axes[1])
                    self._exc_latch = not self._exc_latch
                    debug_str = "LATCHED" if self._exc_latch else "NOT LATCHED"
                    rospy.loginfo(f"EXC {debug_str}")

                if self._exc_latch:
                    effort_msg.excavate = self._exc_latch_val
                else:
                    effort_msg.excavate = constrain(joy.axes[1])  # left stick

                effort_msg.lead_screw = constrain(joy.axes[4])  # right stick

            if self._exc_latch:  # keeps exc latched even when switching to drive mode
                effort_msg.excavate = self._exc_latch_val

            # right bumpers
            effort_msg.lin_act = int(joy.buttons[4] - joy.buttons[5]) * 127

            # right arrows
            effort_msg.deposit = int(joy.axes[6]) * 127

            self._effort_msg = effort_msg

    def loop(self):
        if not self._autonomy:
            self._effort_pub_.publish(self._effort_msg)

    def stop(self):
        self._effort_msg.left_drive = 0
        self._effort_msg.right_drive = 0
        self._effort_msg.excavate = 0
        self._effort_msg.lin_act = 0
        self._effort_msg.lead_screw = 0
        self._effort_msg.deposit = 0

        self._exc_latch_val = 0
        self._exc_latch = True


if __name__ == "__main__":
    rospy.init_node("manual_controller_node")

    man_ctrl = ManualController()
    r = rospy.Rate(20)

    while not rospy.is_shutdown():
        man_ctrl.loop()
        r.sleep()

    rospy.spin()
