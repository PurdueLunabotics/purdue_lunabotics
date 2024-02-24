import rospy

import ascent
import escape
import init_mapping
from geometry_msgs.msg import Twist
from apriltag_ros.msg import AprilTagDetectionArray
from lunabot_msgs.msg import RobotEffort, RobotSensors

class State:
    drive_left_curr = 0
    drive_right_curr = 0
    drive_left_vel = 0
    drive_right_vel = 0

class Effort:
    cmd_left = 0
    cmd_right = 0

def robot_state_cb(self, msg):
    global robot_state
    robot_state.drive_left_curr = msg.drive_left_curr
    robot_state.drive_right_curr = msg.drive_right_curr
    robot_state.dep_curr = msg.dep_curr

def eff_cb(self, msg):
	global robot_effort
	robot_effort.cmd_left = msg.drive_left
	robot_effort.cmd_right = msg.drive_right

def apritag_cb(self, msg):
    global foundTag
    if len(msg.detections) != 0:
        foundTag = True
    else:
        foundTag = False

foundTag = False
robot_state = State()
robot_effort = Effort()

def main():
    rospy.init_node('behavior_node')

    effort_pub = rospy.Publisher("/effort", RobotEffort, queue_size=1)
    vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    
    rospy.Subscriber("/d455_front/camera/color/tag_detections", AprilTagDetectionArray, apritag_cb)
    rospy.Subscriber("/sensors", RobotSensors, robot_state_cb)
    rospy.Subscriber("/effort", RobotEffort, eff_cb)

    #create interrupt logic here!
    # rospy.is_shutdown():
    # stuck
    # map change
    # overcurrent

    # notes on interrupts: if no functions use rospy.sleep, then they can all 
    # just check for the kill signal at the end of every loop they go through. 
    # that can cause them to return false or the error in an enum (need to decide)
    # and that can be handled by the main process before continuing. 
    # How to pick up where we were before? Current state enum flag?
    # also how to cleanly handle the functions returning different values?
    # and picking back up from somewhere new specificed by the enum flag?

    #startup stuff here
    ascent.main()
    init_mapping.main()

    while(True):
        #traverse to mining zone

        #plunge

        #trench

        ascent.main()

        #berm plan

        #go to berm

        #deposit

        #post_deposit

        ascent.main() #why?

        #plan to mining zone