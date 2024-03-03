import rospy
from enum import Enum, auto
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
from lunabot_msgs.msg import RobotEffort, RobotSensors, RobotErrors, Behavior
from std_msgs.msg import Bool

import ascent
import find_apriltag
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
        self.goal_publisher = rospy.Publisher("/goal", PoseStamped, queue_size=1, latch=True)
        self.zone_visual_publisher = rospy.Publisher("/zone_visual", Path, queue_size=1, latch=True)

        self.current_state = States.ASCENT_INIT

        self.start_apriltag: AprilTagDetection = AprilTagDetection()

        self.mining_zone = None
        self.berm_zone = None

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

        self.current_state = States.TRAVERSAL_MINE

        # Translate the apriltag into the odom frame
        apriltag_pose_in_odom: PoseStamped = find_apriltag_module.convert_to_odom_frame(self.start_apriltag)

        # Find the mininz/berm zones in the odom frame
        self.mining_zone: self.Zone = self.find_mining_zone(apriltag_pose_in_odom)
        self.berm_zone: self.Zone = self.find_berm_zone(apriltag_pose_in_odom)

        # Set a goal to the mining zone and publish it
        mining_goal = PoseStamped()
        mining_goal.pose.position.x = self.mining_zone.bottom_right[0]
        mining_goal.pose.position.y = self.mining_zone.bottom_right[1]
        mining_goal.pose.position.z = 0

        mining_goal.header.stamp = rospy.Time.now()
        mining_goal.header.frame_id = "odom"

        self.goal_publisher.publish(mining_goal)

        # This visualizes the given zone as a red square (visible in rviz)
        self.visualize_zone(self.mining_zone)

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
                    
                    # Detect when reached mining zone
                    self.current_state = States.PLUNGE
                
                # Lower linear actuators and begin spinning excavation
                if (self.current_state == States.PLUNGE):
                    rospy.loginfo("State: Plunging")

                    traversal_message.data = False
                    self.traversal_publisher.publish(traversal_message)

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
                        
                    # Set goal to berm
                    self.current_state = States.TRAVERSAL_BERM

                # Drive to berm area
                if (self.current_state == States.TRAVERSAL_BERM):
                    # Enable traversal (to berm)
                    rospy.loginfo("State: Traversal")

                    traversal_message.data = True
                    self.traversal_publisher.publish(traversal_message)

                    # Detect when reached berm
                    self.current_state = States.ALIGN
            
                # Align with an apriltag at the berm
                if (self.current_state == States.ALIGN):
                    rospy.loginfo("State: Alignment")

                    traversal_message.data = False
                    self.traversal_publisher.publish(traversal_message)

                    self.current_state = States.DEPOSIT
                    # Alignment
            
                # Deposit regolith w/ auger
                if (self.current_state == States.DEPOSIT):
                    
                    deposition_status = deposition_module.deposit()

                    if deposition_status == False:
                        break

                    self.current_state = States.TRAVERSAL_MINE

                # Set goal to mining zone
            
            # This block runs when we have an interrupt (some kind of error)
            problem = interrupts.check_for_interrupts()
            if problem == interrupts.Errors.ROS_ENDED:
                #simply exit this loop and the whole program
                break
            elif problem == interrupts.Errors.OVERCURRENT:
                #TODO: what goes here?
                pass
            elif problem == interrupts.Errors.STUCK:
                #if the robot is stuck, unstick it
                escape_module.unstickRobot()

    class Zone:
        def __init__(self, top_left: 'tuple[float]', top_right: 'tuple[float]', bottom_left: 'tuple[float]', bottom_right: 'tuple[float]'):
            # tuples are (x, y) where x is left-right and y is bottom-top
            self.top_left = top_left
            self.top_right = top_right
            self.bottom_left = bottom_left
            self.bottom_right = bottom_right

            self.middle = ((top_left[0] + top_right[0]) / 2, (top_left[1] + bottom_left[1]) / 2)

    def visualize_zone(self, zone: Zone):
        # Make a path containing all of the corners of the zone. Make it into a path, and use the self.publisher to publish it.
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "odom"

        path.poses.append(PoseStamped())
        path.poses[-1].pose.position.x = zone.top_left[0]
        path.poses[-1].pose.position.y = zone.top_left[1]

        path.poses.append(PoseStamped())
        path.poses[-1].pose.position.x = zone.top_right[0]
        path.poses[-1].pose.position.y = zone.top_right[1]

        path.poses.append(PoseStamped())
        path.poses[-1].pose.position.x = zone.bottom_right[0]
        path.poses[-1].pose.position.y = zone.bottom_right[1]

        path.poses.append(PoseStamped())
        path.poses[-1].pose.position.x = zone.bottom_left[0]
        path.poses[-1].pose.position.y = zone.bottom_left[1]

        path.poses.append(PoseStamped())
        path.poses[-1].pose.position.x = zone.top_left[0]
        path.poses[-1].pose.position.y = zone.top_left[1]

        self.zone_visual_publisher.publish(path)

    def find_mining_zone(self, apriltag_pose_in_odom: PoseStamped)->Zone:

        #TODO decide how to deal with UCF arena

        DIST_X = 3.88 # In meters, the distance from the leftmost wall to the left border of the mining zone
        # KSC = 3.88
        LENGTH_X = 3 # In meters, the length of the mining zone (left to right)
        # KSC = 3

        DIST_Y = 2 # In meters, the distance from the bottom-most wall to the bottom border of the mining zone
        # KSC = 2
        LENGTH_Y = 3 # In meters, the length of the mining zone (bottom to top)
        # KSC = 3

        roll, pitch, yaw  = euler_from_quaternion([apriltag_pose_in_odom.pose.orientation.x, apriltag_pose_in_odom.pose.orientation.y, apriltag_pose_in_odom.pose.orientation.z, apriltag_pose_in_odom.pose.orientation.w])
        yaw += math.pi / 2
        y_yaw = yaw + math.pi / 2

        top_left = (apriltag_pose_in_odom.pose.position.x + math.cos(yaw) * (DIST_X), apriltag_pose_in_odom.pose.position.y + math.sin(y_yaw) * (DIST_Y + LENGTH_Y))
        top_right = (apriltag_pose_in_odom.pose.position.x + math.cos(yaw) * (DIST_X + LENGTH_X), apriltag_pose_in_odom.pose.position.y + math.sin(y_yaw) * (DIST_Y + LENGTH_Y))
        bottom_left = (apriltag_pose_in_odom.pose.position.x + math.cos(yaw) * (DIST_X), apriltag_pose_in_odom.pose.position.y + math.sin(y_yaw) * (DIST_Y))
        bottom_right = (apriltag_pose_in_odom.pose.position.x + math.cos(yaw) * (DIST_X + LENGTH_X), apriltag_pose_in_odom.pose.position.y + math.sin(y_yaw) * (DIST_Y))

        return self.Zone(top_left, top_right, bottom_left, bottom_right)
    
    def find_berm_zone(self, apriltag_pose_in_odom: PoseStamped)->Zone:

        # TODO update these, they are not exact
        DIST_X = 3.88 + 1
        # KSC = ~4.88  UCF ~7.04
        LENGTH_X = 2
        # KSC = ~2   UCF ~1.6

        DIST_Y = 0.1
        # KSC = ~0.1   UCF ~3.37
        LENGTH_Y = 1
        # KSC = ~1    UCF ~1.8

        roll, pitch, yaw  = euler_from_quaternion([apriltag_pose_in_odom.pose.orientation.x, apriltag_pose_in_odom.pose.orientation.y, apriltag_pose_in_odom.pose.orientation.z, apriltag_pose_in_odom.pose.orientation.w])
        yaw += math.pi / 2
        y_yaw = yaw + math.pi / 2

        top_left = (apriltag_pose_in_odom.pose.position.x + math.cos(yaw) * (DIST_X), apriltag_pose_in_odom.pose.position.y + math.sin(y_yaw) * (DIST_Y + LENGTH_Y))
        top_right = (apriltag_pose_in_odom.pose.position.x + math.cos(yaw) * (DIST_X + LENGTH_X), apriltag_pose_in_odom.pose.position.y + math.sin(y_yaw) * (DIST_Y + LENGTH_Y))
        bottom_left = (apriltag_pose_in_odom.pose.position.x + math.cos(yaw) * (DIST_X), apriltag_pose_in_odom.pose.position.y + math.sin(y_yaw) * (DIST_Y))
        bottom_right = (apriltag_pose_in_odom.pose.position.x + math.cos(yaw) * (DIST_X + LENGTH_X), apriltag_pose_in_odom.pose.position.y + math.sin(y_yaw) * (DIST_Y))

        return self.Zone(top_left, top_right, bottom_left, bottom_right)


        
            

if __name__ == "__main__":
    behavior = Behavior()
    behavior.behavior_loop()
    rospy.spin()