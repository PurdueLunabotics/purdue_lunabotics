import rospy
from enum import Enum, auto()

from geometry_msgs.msg import Twist
from apriltag_ros.msg import AprilTagDetectionArray
from lunabot_msgs.msg import RobotEffort, RobotSensors, RobotErrors, Behavior
from std_msgs.msg import Bool

import ascent
import escape
import find_apriltag
import interrupts

class states(Enum):
    ASCENT = auto()
    FIND_TAG = auto()
    TRAVERSAL_MINE = auto()
    PLUNGE = auto()
    TRENCH = auto()
    TRAVERSAL_BERM = auto()
    DEPOSIT = auto()

'''
A class that controls the main behavior of the robot, aiming for a cycle of autonomous mining and berm depositing
Consists of a variety of states, most of which are imported python modules. Publishes robot effort and cmd_vel,
along with the various submodules (which share publishers when possible). For autonomous driving, the class
publishes a boolean state that enables or disables MPC.
'''
class Behavior:

    def robot_state_callback(self, msg: RobotSensors):
        self.robot_state = msg

    def effort_callback(self, msg: RobotEffort):
        self.robot_effort = msg

    def errors_callback(self, msg: RobotErrors):
        self.robot_errors = msg

    def __init__(self):
        self.robot_state: RobotSensors = RobotSensors()
        self.robot_effort: RobotEffort = RobotEffort()
        self.robot_errors: RobotErrors = RobotErrors()

        self.effort_publisher = rospy.Publisher("/effort", RobotEffort, queue_size=1, latch=True)
        self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1, latch=True)
        self.traversal_publisher = rospy.Publisher("/behavior/traversal_enabled", Bool, queue_size=1, latch=True)

        # TODO change to parameters, determine which are needed
        rospy.Subscriber("/sensors", RobotSensors, self.robot_state_callback)
        rospy.Subscriber("/effort", RobotEffort, self.effort_callback)
        rospy.Subscriber("/errors", RobotErrors, self.errors_callback)

        rospy.init_node('behavior_node')

    """
    The main method of the class: enables autonomous behavior. Starts up with a few states,
    then goes in a loop of mining/deposition.
    """
    def behavior_loop(self):

        # notes on interrupts: if no functions use rospy.sleep, then they can all 
        # just check for the kill signal at the end of every loop they go through. 
        # that can cause them to return false or the error in an enum (need to decide)
        # and that can be handled by the main process before continuing. 
        # How to pick up where we were before? Current state enum flag?
        # also how to cleanly handle the functions returning different values?
        # and picking back up from somewhere new specificed by the enum flag?

        # Startup:

        # disable traversal to begin
        traversal_message = Bool()
        traversal_message.data = False
        self.traversal_publisher.publish(traversal_message)

        rospy.loginfo("State: Ascent")
        ascent_module = ascent.Ascent(self.effort_publisher)
        ascent_module.raiseLinearActuators()

        rospy.loginfo("State: Find AprilTag")
        find_apriltag_module = find_apriltag.FindAprilTag(self.velocity_publisher)
        find_apriltag_module.find_apriltag()

        # Determine positions of mining and berm zones from apriltag

        # Set goal to mining zone

        while(not rospy.is_shutdown()):
            # Enable traversal (to mining zone)
            rospy.loginfo("State: Traversal")
            traversal_message.data = True
            self.traversal_publisher.publish(traversal_message)

            # Detect when reached mining zone

            rospy.loginfo("State: Plunging")
            traversal_message.data = False
            self.traversal_publisher.publish(traversal_message)

            # plunge

            # trench / mine

            rospy.loginfo("State: Ascent")
            ascent_module.raiseLinearActuators()

            # Set goal to berm

            # Enable traversal (to berm)
            rospy.loginfo("State: Traversal")
            traversal_message.data = True
            self.traversal_publisher.publish(traversal_message)

            # Detect when reached berm

            rospy.loginfo("State: Alignment")
            traversal_message.data = False
            self.traversal_publisher.publish(traversal_message)

            # Alignment
            
            # Deposit

            rospy.loginfo("State: Ascent")
            ascent_module.raiseLinearActuators() # TODO why do we do this?

            # Set goal to mining zone

if __name__ == "__main__":
    behavior = Behavior()
    behavior.behavior_loop()
    rospy.spin()