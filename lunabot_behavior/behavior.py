import rospy
from enum import Enum, auto

from geometry_msgs.msg import Twist
from apriltag_ros.msg import AprilTagDetectionArray
from lunabot_msgs.msg import RobotEffort, RobotSensors, RobotErrors, Behavior
from std_msgs.msg import Bool

import ascent
import escape
import find_apriltag
import interrupts

#TODO: differentiate between different ascent states?
class States(Enum):
    ASCENT = auto()
    FIND_TAG = auto()
    TRAVERSAL_MINE = auto()
    PLUNGE = auto()
    TRENCH = auto()
    TRAVERSAL_BERM = auto()
    ALIGN = auto()
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

        self.current_state = States()
        self.current_state = States.ASCENT

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
        self.current_state = States.ASCENT
        ascent_module = ascent.Ascent(self.effort_publisher)
        ascent_module.raiseLinearActuators()

        #TODO: What do we do if there's a failure here? More loops?
        rospy.loginfo("State: Find AprilTag")
        self.current_state = States.FIND_TAG
        find_apriltag_module = find_apriltag.FindAprilTag(self.velocity_publisher)
        find_apriltag_module.find_apriltag()
        self.current_state = States.TRAVERSAL_MINE
        # Determine positions of mining and berm zones from apriltag

        # Set goal to mining zone

        #This loop always running until we end the program
        while(not rospy.is_shutdown()):
            #This loop is running while things are fine. Break out if interrupts
            while (interrupts.main() == interrupts.Errors.FINE):
                #off to the mines
                if (self.current_state == States.TRAVERSAL_MINE):
                    # Enable traversal (to mining zone)
                    rospy.loginfo("State: Traversal")
                    traversal_message.data = True
                    self.traversal_publisher.publish(traversal_message)
                    # Detect when reached mining zone
                    self.current_state = States.PLUNGE
                
                if (self.current_state == States.PLUNGING):
                    rospy.loginfo("State: Plunging")
                    traversal_message.data = False
                    self.traversal_publisher.publish(traversal_message)
                    self.current_state = States.TRENCH

                if (self.current_state == States.TRENCH):
                    # trench / mine
                    self.current_state = States.ASCENT

                if (self.current_state == States.ASCENT):
                    rospy.loginfo("State: Ascent")
                    if not ascent_module.raiseLinearActuators():
                        break
                    # Set goal to berm
                    self.current_state = States.TRAVERSAL_BERM

                if (self.current_state == States.TRAVERSAL_BERM):
                    # Enable traversal (to berm)
                    rospy.loginfo("State: Traversal")
                    traversal_message.data = True
                    self.traversal_publisher.publish(traversal_message)
                    # Detect when reached berm
                    self.current_state = States.ALIGN
            
                if (self.current_state == States.ALIGN):
                    rospy.loginfo("State: Alignment")
                    traversal_message.data = False
                    self.traversal_publisher.publish(traversal_message)
                    self.current_state = States.DEPOSIT
                    # Alignment
            
                if (self.current_state == States.DEPOSIT):
                    # Deposit
                    self.current_state = States.ASCENT

                if (self.current_state == States.ASCENT):
                    rospy.loginfo("State: Ascent")
                    if not ascent_module.raiseLinearActuators(): # TODO why do we do this?
                        break
                    self.current_state = States.TRAVERSAL_MINE
                # Set goal to mining zone
            
            #only get here if we have an interrupt of some kind
            problem = interrupts.main()
            if problem == interrupts.Errors.ROS_ENDED:
                #simply exit this loop and the whole program
                break
            elif problem == interrupts.Errors.OVERCURRENT:
                #TODO: what goes here?
                pass
            elif problem == interrupts.Errors.STUCK:
                #if the robot is stuck, unstick it
                escape_module = escape.Escape(self.velocity_publisher)
                escape_module.unstickRobot()
            

if __name__ == "__main__":
    behavior = Behavior()
    behavior.behavior_loop()
    rospy.spin()