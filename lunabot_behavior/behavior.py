#!/usr/bin/env python3

import rospy
from enum import Enum, auto
import math

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
from lunabot_msgs.msg import RobotEffort, RobotSensors, RobotErrors, Behavior
from std_msgs.msg import Bool

import ascent
import find_apriltag
import zones
import plunge
import deposition
import interrupts
import escape

class States(Enum):
    ASCENT_INIT = auto()
    FIND_TAG = auto()
    TRAVERSAL_MINE = auto()
    PLUNGE = auto()
    TRENCH = auto()
    ASCENT_MINING = auto()
    TRAVERSAL_BERM = auto()
    ALIGN = auto()
    DEPOSIT = auto()

class Behavior:
    '''
    A class that controls the main behavior of the robot, aiming for a cycle of autonomous mining and berm depositing
    Consists of a variety of states, most of which are imported python modules. Publishes robot effort and cmd_vel,
    along with the various submodules (which share publishers when possible). For autonomous driving, the class
    publishes a boolean state that enables or disables MPC.
    '''

    def robot_state_callback(self, msg: RobotSensors):
        self.robot_state = msg

    def effort_callback(self, msg: RobotEffort):
        self.robot_effort = msg

    def errors_callback(self, msg: RobotErrors):
        self.robot_errors = msg

    def odom_callback(self, msg: Odometry):
        self.robot_odom = msg

    def __init__(self):

        rospy.init_node('behavior_node')

        self.robot_state: RobotSensors = RobotSensors()
        self.robot_effort: RobotEffort = RobotEffort()
        self.robot_errors: RobotErrors = RobotErrors()
        self.robot_odom: Odometry = Odometry()

        self.effort_publisher = rospy.Publisher("/effort", RobotEffort, queue_size=1, latch=True)
        self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1, latch=True)
        self.traversal_publisher = rospy.Publisher("/behavior/traversal_enabled", Bool, queue_size=1, latch=True)
        self.goal_publisher = rospy.Publisher("/goal", PoseStamped, queue_size=1, latch=True)
        self.zone_visual_publisher = rospy.Publisher("/zone_visual", Path, queue_size=1, latch=True)

        self.current_state = States.ASCENT_INIT

        self.start_apriltag: AprilTagDetection = AprilTagDetection()

        self.mining_zone = None
        self.berm_zone = None

        self.rate = rospy.Rate(1) #hz

        # TODO change to parameters, determine which are needed
        rospy.Subscriber("/sensors", RobotSensors, self.robot_state_callback)
        rospy.Subscriber("/effort", RobotEffort, self.effort_callback)
        rospy.Subscriber("/errors", RobotErrors, self.errors_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)


    def is_close_to_goal(self, goal: PoseStamped) -> bool:
        """
        Checks if the robot is close to a given goal
        """

        THRESHOLD = 0.15 # meters

        x = self.robot_odom.pose.pose.position.x
        y = self.robot_odom.pose.pose.position.y

        goal_x = goal.pose.position.x
        goal_y = goal.pose.position.y

        distance = math.sqrt((x - goal_x)**2 + (y - goal_y)**2)

        rospy.logdebug("Behavior: Distance to goal: " + str(distance))

        return distance < THRESHOLD


    def behavior_loop(self):
        """
        The main method of the class: enables autonomous behavior. Starts up with a few states,
        then goes in a loop of mining/deposition.
        """

        # Initialize all of the modules (before the loop)
        ascent_module = ascent.Ascent(self.effort_publisher)
        find_apriltag_module = find_apriltag.FindAprilTag(self.velocity_publisher)
        plunge_module = plunge.Plunge(self.effort_publisher)

        deposition_module = deposition.Deposition(self.effort_publisher)

        escape_module = escape.Escape(self.velocity_publisher)

        # Startup:

        # disable traversal to begin
        traversal_message = Bool()
        traversal_message.data = False
        self.traversal_publisher.publish(traversal_message)

        # Raise linear actuators
        rospy.loginfo("State: Ascent")
        self.current_state = States.ASCENT_INIT

        ascent_status = ascent_module.raise_linear_actuators()
        if ascent_status == False: # Robot error
            pass # TODO implement the error functions here

        
        # Spin until we find the start apriltag
        rospy.loginfo("State: Find AprilTag")
        self.current_state = States.FIND_TAG

        apriltag_status = find_apriltag_module.find_apriltag()

        if apriltag_status == "Error": # Robot error
            pass # TODO implement the error functions here
        elif apriltag_status == None: # Could not find apriltag
            pass #TODO pick what to do here
        else:
            self.start_apriltag = apriltag_status

        # Translate the apriltag into the odom frame
        apriltag_pose_in_odom: PoseStamped = find_apriltag_module.convert_to_odom_frame(self.start_apriltag)

        # Find the mininz/berm zones in the odom frame
        self.mining_zone: zones.Zone = zones.find_mining_zone(apriltag_pose_in_odom)
        self.berm_zone: zones.Zone = zones.find_berm_zone(apriltag_pose_in_odom)

        # Set a goal to the mining zone and publish it
        mining_goal = PoseStamped()
        mining_goal.pose.position.x = self.mining_zone.middle[0]
        mining_goal.pose.position.y = self.mining_zone.middle[1]
        mining_goal.pose.position.z = 0

        mining_goal.header.stamp = rospy.Time.now()
        mining_goal.header.frame_id = "odom"


        # create a goal for the berm
        berm_goal = PoseStamped()
        berm_goal.pose.position.x = self.berm_zone.middle[0]
        berm_goal.pose.position.y = self.berm_zone.middle[1]
        berm_goal.pose.position.z = 0

        berm_goal.header.stamp = rospy.Time.now()
        berm_goal.header.frame_id = "odom"

        self.current_state = States.TRAVERSAL_MINE

        print(mining_goal)
        self.goal_publisher.publish(mining_goal)

        # This visualizes the given zone as a red square (visible in rviz)
        self.mining_zone.visualize_zone(self.zone_visual_publisher)

        #This loop always running until we end the program
        while(not rospy.is_shutdown()):

            #This loop is running while things are fine. Break out if interrupts
            while (interrupts.check_for_interrupts() == interrupts.Errors.FINE):

                # Drive to the mining area
                if (self.current_state == States.TRAVERSAL_MINE):
                    # Enable traversal (to mining zone)
                    rospy.loginfo("State: Traversal")
                    traversal_message.data = True
                    self.traversal_publisher.publish(traversal_message)


                    while (not self.is_close_to_goal(mining_goal)):
                        # Wait until close enough
                        self.rate.sleep()
                        if (interrupts.check_for_interrupts() != interrupts.Errors.FINE):
                            break

                    traversal_message.data = False
                    self.traversal_publisher.publish(traversal_message)
                    
                    self.current_state = States.PLUNGE
                
                # Lower linear actuators and begin spinning excavation
                if (self.current_state == States.PLUNGE):
                    rospy.loginfo("State: Plunging")

                    plunge_status = plunge_module.plunge()
                    if plunge_status == False:
                        break

                    self.current_state = States.TRENCH

                # Mine, and possibly drive while mining
                if (self.current_state == States.TRENCH):
                    # trench / mine
                    self.current_state = States.ASCENT_MINING

                # Raise linear actuators
                if (self.current_state == States.ASCENT_MINING):
                    rospy.loginfo("State: Ascent")
                    ascent_status = ascent_module.raise_linear_actuators()

                    if ascent_status == False: # Robot error
                        break
                        
                    # Set a goal to the berm zone and publish it
                    berm_goal.header.stamp = rospy.Time.now()
                    self.goal_publisher.publish(berm_goal)
                    self.berm_zone.visualize_zone(self.zone_visual_publisher)
                    
                    self.current_state = States.TRAVERSAL_BERM

                # Drive to berm area
                if (self.current_state == States.TRAVERSAL_BERM):
                    # Enable traversal (to berm)
                    rospy.loginfo("State: Traversal")

                    traversal_message.data = True
                    self.traversal_publisher.publish(traversal_message)

                    while (not self.is_close_to_goal(berm_goal)):
                        # Wait until close enough
                        self.rate.sleep()
                        if (interrupts.check_for_interrupts() != interrupts.Errors.FINE):
                            break

                    traversal_message.data = False
                    self.traversal_publisher.publish(traversal_message)

                    # Detect when reached berm
                    self.current_state = States.ALIGN
            
                # Align with an apriltag at the berm
                if (self.current_state == States.ALIGN):
                    rospy.loginfo("State: Alignment")
                    # Alignment

                    self.current_state = States.DEPOSIT
            
                # Deposit regolith w/ auger
                if (self.current_state == States.DEPOSIT):
                    
                    deposition_status = deposition_module.deposit()

                    if deposition_status == False:
                        break

                    # Set a goal to the mining zone and publish it
                    mining_goal.header.stamp = rospy.Time.now()
                    self.goal_publisher.publish(mining_goal)
                    self.mining_zone.visualize_zone(self.zone_visual_publisher)

                    self.current_state = States.TRAVERSAL_MINE

                self.rate.sleep()
                
            
            # This block runs when we have an interrupt (some kind of error)
            problem = interrupts.check_for_interrupts()
            if problem == interrupts.Errors.ROS_ENDED:
                #simply exit this loop and the whole program
                break
            elif problem == interrupts.Errors.MANUAL_STOP:
                # also exit the loop/stop autonomy
                break
            elif problem == interrupts.Errors.OVERCURRENT:
                #TODO: what goes here?
                pass
            elif problem == interrupts.Errors.STUCK:
                #if the robot is stuck, unstick it
                escape_module.unstickRobot()
         

if __name__ == "__main__":
    behavior = Behavior()
    behavior.behavior_loop()
    rospy.spin()
