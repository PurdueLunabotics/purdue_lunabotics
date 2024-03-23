#!/usr/bin/env python3
import rospy

from sensor_msgs.msg import Joy
from lunabot_msgs.msg import RobotEffort

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

# Take an input from -1 to 1, and convert it to an 8 bit int from -127 to 127
def constrain(joy_in: float):
    return np.int8(np.clip(joy_in, -1, 1) * 127)

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

"""
Manual Controller
Publishes an effort message to control the robot.
Control Scheme:
 (tank drive)
- Left stick: left wheels
- Right stick: right wheels
- Right trigger Excavation forwards (pick up dirt)
- Left trigger: Excavation backwards
- D-pad up/down: Linear actuator control (move excavation system up/down)
- Y button: Latch/unlatch excavation speed
    - When latched, the excavation speed will stay at its last speed until unlatched
- X button: Switch between forwards and backwards driving
    - Forwards is defined as leading with excavation
- B button: Deposition (spin auger)
- Start button: Stop the robot
"""
class ManualController:

    def __init__(self):
        self._joy_sub = rospy.Subscriber("joy", Joy, self.joy_callback)
        self._effort_pub_ = rospy.Publisher("effort", RobotEffort, queue_size=1)
        self._effort_msg = RobotEffort()

        self.last_joy = Joy()

        self._driving_mode = "Forwards"
        self._autonomy = False

        self._latched_excavation_speed = 0
        self._excavation_is_latched = False

        self.DEPOSITION_SPEED = 127

        self.hasReadRT = False
        self.hasReadLT = False

        self.stop()

    def joy_callback(self, joy):
        # X button: Switch between driving forwards and backwards
        if joy.buttons[Buttons.X.value] == 1 and self.last_joy.buttons[Buttons.X.value] == 0:
            if (self._driving_mode == "Forwards"):
                self._driving_mode = "Backwards"
            else:
                self._driving_mode = "Forwards"

            rospy.loginfo(f"Driving Direction: {self._driving_mode}")

        # Start button: Stop the robot (Pause)
        if joy.buttons[Buttons.START.value] == 1:
            self.stop()
            rospy.loginfo("Manual Control: Stopped")
        else:
            effort_msg = RobotEffort()

            effort_msg.left_drive = 0
            effort_msg.right_drive = 0

            effort_msg.excavate = 0

            # Set the drive effort to the left and right stick vertical axes (Tank Drive)
            if self._driving_mode == "Forwards":
                effort_msg.left_drive = constrain(joy.axes[Axes.L_STICK_VERTICAL.value])
                effort_msg.right_drive = constrain(joy.axes[Axes.R_STICK_VERTICAL.value])
            else:
                effort_msg.left_drive = -1 * constrain(joy.axes[Axes.R_STICK_VERTICAL.value])
                effort_msg.right_drive = -1 * constrain(joy.axes[Axes.L_STICK_VERTICAL.value])
            
            # If not latched, use the trigger axis to control the excavation speed. Otherwise, use the latched speed
            if self._excavation_is_latched:
                effort_msg.excavate = self._latched_excavation_speed
            else:
                # Change the range of the trigger axis from [1, -1] to [0, 1]
                if (joy[Axes.RIGHT_TRIGGER.value] == 0) {
                    right_trigger_axis_normalized = 0
                } else {
                    right_trigger_axis_normalized = (-joy.axes[Axes.RIGHT_TRIGGER.value] + 1) / 2
                }
                
                if (joy[Axes.LEFT_TRIGGER.value] == 0) {
                    left_trigger_axis_normalized = 0
                } else {
                    left_trigger_axis_normalized = (-joy.axes[Axes.LEFT_TRIGGER.value] + 1) / 2
                }                
                '''
                if not self.hasReadLT:
                    left_trigger_axis_normalized = 0
                    if joy.axes[Axes.LEFT_TRIGGER.value] != 0:
                        self.hasReadLT = True

                if not self.hasReadRT:
                    right_trigger_axis_normalized = 0
                    if joy.axes[Axes.RIGHT_TRIGGER.value] != 0:
                        self.hasReadRT = True
                '''

                # Take priority for right trigger. If it is nearly zero, use the left trigger instead
                if (right_trigger_axis_normalized <= 0.01):
                    effort_msg.excavate = -1 * constrain(left_trigger_axis_normalized)
                else:
                    effort_msg.excavate = constrain(right_trigger_axis_normalized)

            # Y button: latch excavation speed. This keeps excavation at the same speed until the latch is released
            if joy.buttons[Buttons.Y.value] == 1 and self.last_joy.buttons[Buttons.Y.value] == 0:  
                if (self._excavation_is_latched):
                    self._excavation_is_latched = False
                    rospy.loginfo("Excavation Released")
                else:
                    self._excavation_is_latched = True
                    self._latched_excavation_speed = effort_msg.excavate
                    rospy.loginfo(f"Excavation Latched at {self._latched_excavation_speed}")

            # Dpad up/down - control linear actuators
            effort_msg.lin_act = constrain(joy.axes[Axes.DPAD_VERTICAL.value])

            if (joy.buttons[Buttons.B.value] == 1):
                effort_msg.deposit = self.DEPOSITION_SPEED

            self._effort_msg = effort_msg
            self.last_joy = joy

    def loop(self):
        if not self._autonomy:
            self._effort_pub_.publish(self._effort_msg)

    def stop(self):
        self._effort_msg.left_drive = 0
        self._effort_msg.right_drive = 0
        self._effort_msg.excavate = 0
        self._effort_msg.lin_act = 0
        self._effort_msg.deposit = 0

        self._exc_latch_val = 0
        self._exc_latch = True

        self._effort_pub_.publish(self._effort_msg)


if __name__ == "__main__":
    rospy.init_node("manual_controller_node")

    man_ctrl = ManualController()
    r = rospy.Rate(20)

    while not rospy.is_shutdown():
        man_ctrl.loop()
        r.sleep()

    rospy.spin()
