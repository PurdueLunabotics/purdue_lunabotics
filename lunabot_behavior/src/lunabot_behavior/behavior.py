#!/usr/bin/env python3

import rospy
import math
import numpy as np

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist, PoseStamped, PolygonStamped
from nav_msgs.msg import Path, Odometry
from lunabot_msgs.msg import RobotSensors, RobotErrors, Behavior
from std_msgs.msg import Bool, Int8, Int32

from lunabot_behavior.linear_actuators import LinearActuatorManager
from lunabot_behavior.alignment import AlignmentController
from lunabot_behavior.exdep import ExdepController
from geometry_msgs.msg import Twist

import zones
import excavate
import deposition

class Behavior:
    '''
    A class that controls the main behavior of the robot, aiming for a cycle of autonomous mining and berm depositing
    Consists of a variety of states, most of which are imported python modules. Publishes robot effort and cmd_vel,
    along with the various submodules (which share publishers when possible). For autonomous driving, the class
    publishes a boolean state that enables or disables traversal
    '''

    def odom_callback(self, msg: Odometry):
        self.robot_odom = msg
    
    def cmd_vel_callback(self, msg: Twist):
        self.cmd_vel = msg
        
    def is_stopped(self) -> bool:
        return self.cmd_vel != None and (self.cmd_vel.angular.z == 0 and self.cmd_vel.linear.x == 0)

    def apriltag_pose_callback(self, msg: PoseStamped):
        self.apriltag_pose_in_odom = msg
        
        # Find the mining/berm zones in the odom frame
        self.mining_zone: zones.Zone = zones.find_mining_zone(self.apriltag_pose_in_odom, self.is_sim)
        self.berm_zone: zones.Zone = zones.find_berm_zone(self.apriltag_pose_in_odom, self.is_sim)

    def __init__(self):

        rospy.init_node('behavior_node')

        self.robot_odom: Odometry = Odometry()
        self.apriltag_pose_in_odom: PoseStamped = None

        self.lin_act_publisher = rospy.Publisher("/lin_act", Int32, queue_size=1, latch=True)
        self.excavate_publisher = rospy.Publisher("/excavate", Int32, queue_size=1, latch=True)
        self.deposition_publisher = rospy.Publisher("/deposition", Int32, queue_size=1, latch=True)

        self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1, latch=True)
        self.velocity_subscriber = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        self.traversal_publisher = rospy.Publisher("/behavior/traversal_enabled", Bool, queue_size=1, latch=True)
        self.goal_publisher = rospy.Publisher("/goal", PoseStamped, queue_size=1, latch=True)

        self.backwards_publisher = rospy.Publisher("/traversal/backwards", Bool, queue_size=1, latch=True)

        self.apriltag_enabled_publisher = rospy.Publisher("/apriltag/enabled", Bool, queue_size=1, latch=True)
        self.apriltag_pose_publisher = rospy.Publisher("/apriltag_pose", PoseStamped, queue_size=1, latch=True)

        self.mining_zone = None
        self.berm_zone = None

        self.rate = rospy.Rate(1) #hz

        self.is_sim = rospy.get_param("/is_sim")

        odom_topic = rospy.get_param("/odom_topic")
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback)

        apriltag_topic = rospy.get_param("/apriltag_topic")
        rospy.Subscriber(apriltag_topic, PoseStamped, self.apriltag_pose_callback)

        self.SPIN_SPEED = math.radians(20)  # rad/s
        self.SPIN_TIME = (math.pi / 2) / self.SPIN_SPEED  # seconds, how long to spin at the beginning for 360 degree mapping (only spins 90 degrees)

        self.MAX_APRILTAG_SEARCH_TIME = 30.0  # seconds, how long to search for an apriltag before giving up

        self.APRILTAG_AVERAGING_TIME = 5.0 # seconds, how long the robot will take average apriltag pose for


    def is_close_to_goal(self, goal: PoseStamped) -> bool:
        """
        Checks if the robot is close to a given goal
        """

        THRESHOLD = 0.6 # meters

        x = self.robot_odom.pose.pose.position.x
        y = self.robot_odom.pose.pose.position.y

        goal_x = goal.pose.position.x
        goal_y = goal.pose.position.y

        distance = math.sqrt((x - goal_x)**2 + (y - goal_y)**2)

        rospy.logdebug("Behavior: Distance to goal: " + str(distance))

        return distance < THRESHOLD
    

    def get_pose_average(self, pose_list) -> PoseStamped:
        avg_pose = pose_list[0]

        if (len(pose_list) == 1):
            return avg_pose

        # pose point averages
        pt_x_sum = 0.0
        pt_y_sum = 0.0
        pt_z_sum = 0.0

        # pose quaternion averages
        quat_x_sum = 0.0
        quat_y_sum = 0.0
        quat_z_sum = 0.0
        quat_w_sum = 0.0

        for pose in pose_list:
            pt_x_sum = pt_x_sum + pose.pose.position.x
            pt_y_sum = pt_y_sum + pose.pose.position.y
            pt_z_sum = pt_z_sum + pose.pose.position.z

            quat_x_sum = quat_x_sum + pose.pose.orientation.x
            quat_y_sum = quat_y_sum + pose.pose.orientation.y
            quat_z_sum = quat_z_sum + pose.pose.orientation.z
            quat_w_sum = quat_w_sum + pose.pose.orientation.w

        # update pose point
        avg_pose.pose.position.x = pt_x_sum / len(pose_list)
        avg_pose.pose.position.y = pt_y_sum / len(pose_list)
        avg_pose.pose.position.z = pt_z_sum / len(pose_list)

        # update pose orientation
        avg_pose.pose.orientation.x = quat_x_sum / len(pose_list)
        avg_pose.pose.orientation.y = quat_y_sum / len(pose_list)
        avg_pose.pose.orientation.z = quat_z_sum / len(pose_list)
        avg_pose.pose.orientation.w = quat_w_sum / len(pose_list)

        return avg_pose

    
    
        
        


    def behavior_loop(self):
        """
        The main method of the class: enables autonomous behavior. Starts up with a few states,
        then goes in a loop of mining/deposition.
        """

        self.apriltag_enabled_publisher.publish(True) # enable apriltag detection

        # Initialize all of the modules (before the loop)
        linear_actuators = LinearActuatorManager(self.lin_act_publisher)
        alignment_controller = AlignmentController(self.velocity_publisher)
        exdep_controller = ExdepController(self.excavate_publisher, self.lin_act_publisher,
                                           self.velocity_publisher, self.deposition_publisher,
                                           self.traversal_publisher, self.goal_publisher)
        ####################
        # Startup & apriltag
        ####################

        # disable traversal to begin
        traversal_message = Bool()
        traversal_message.data = False
        self.traversal_publisher.publish(traversal_message)

        # set backwards driving to false
        backwards_message = Bool()
        backwards_message.data = False
        self.traversal_publisher.publish(backwards_message)

        # Raise linear actuators
        rospy.loginfo("Behavior: Raising linear actuator")
        linear_actuators.raise_linear_actuators()

        # Spin for one loop to map environment
        rospy.loginfo("Behavior: Mapping")

        velocity_message = Twist()
        velocity_message.linear.x = 0
        velocity_message.angular.z = self.SPIN_SPEED
        self.velocity_publisher.publish(velocity_message)

        start_time = rospy.get_time()
        while rospy.get_time() - start_time < self.SPIN_TIME:
            self.rate.sleep()

        velocity_message.linear.x = 0
        velocity_message.angular.z = 0
        self.velocity_publisher.publish(velocity_message)

        # if don't have an apriltag yet, spin until you find an apriltag (limit by time)
        rospy.loginfo("Behavior: Find apriltag")

        start_time = rospy.get_time()
        while self.apriltag_pose_in_odom == None and rospy.get_time() - start_time < self.MAX_APRILTAG_SEARCH_TIME:
            velocity_message = Twist()
            velocity_message.linear.x = 0
            velocity_message.angular.z = self.SPIN_SPEED
            self.velocity_publisher.publish(velocity_message)

        rospy.sleep(1)

        velocity_message.linear.x = 0
        velocity_message.angular.z = 0
        self.velocity_publisher.publish(velocity_message)

        # if we still don't have an apriltag, something is wrong, exit
        if self.apriltag_pose_in_odom == None:
            rospy.logerr("Behavior: Could not find apriltag")
            return

        # collect april tag average pose over apriltag averaging time period
        rospy.loginfo("Behavior: April Tag Detected")
        rospy.loginfo("Behavior: Collecting average april tag pose")

        last_apriltag_pose = self.apriltag_pose_in_odom
        apriltag_pose_list = []
        apriltag_pose_list.append(last_apriltag_pose) # make sure list is not empty

        start_time_s = rospy.get_rostime().secs
        while rospy.get_rostime().secs - start_time_s < self.APRILTAG_AVERAGING_TIME:
            # add april tag pose to list of poses to average
            if self.apriltag_pose_in_odom != last_apriltag_pose:
                apriltag_pose_list.append(self.apriltag_pose_in_odom)
                last_apriltag_pose = self.apriltag_pose_in_odom

        # diable apriltag node and wait to make sure it's been disabled and does not publish any more
        rospy.loginfo("Behavior: Avg April Tag Pose determined. Disabling April Tag node")
        avg_apriltag_pose = self.get_pose_average(apriltag_pose_list)
        self.apriltag_enabled_publisher.publish(False)

        rospy.sleep(5)

        self.apriltag_pose_publisher.publish(avg_apriltag_pose)
        
        ####################
        # Traversal
        ####################

        # Set an initial goal in the mining zone and publish it
        mining_goal = PoseStamped()
        mining_goal.pose.position.x = self.mining_zone.middle[0]
        mining_goal.pose.position.y = self.mining_zone.middle[1]

        offset = zones.calc_offset(-0.3, 0, self.apriltag_pose_in_odom, self.is_sim)
        mining_goal.pose.position.x += offset[0]
        mining_goal.pose.position.y += offset[1]
        mining_goal.pose.position.z = 0

        mining_goal.header.stamp = rospy.Time.now()
        mining_goal.header.frame_id = "odom"

        # align to facing 'north' to make traversal easier
        alignment_controller.align_to_angle(np.pi / 2)

        # publish goal
        self.goal_publisher.publish(mining_goal)

        # traverse to goal by enabling traversal
        rospy.loginfo("Behavior: Moving to goal")

        traversal_message.data = True
        self.traversal_publisher.publish(traversal_message)

        # wait until we get close to the goal
        while (not self.is_close_to_goal(mining_goal)):
            # if (self.is_stopped()):
                # self.goal_publisher.publish(mining_goal)
            self.rate.sleep()

        traversal_message.data = False
        self.traversal_publisher.publish(traversal_message)

        ####################
        # Excavation-Deposition cycle
        ####################

        # Assume we've made it to the goal, start the excavation-deposition cycle
        exdep_controller.exdep_loop()
         

if __name__ == "__main__":
    behavior = Behavior()
    behavior.behavior_loop()
    rospy.spin()
