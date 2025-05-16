#!/usr/bin/env python3

import rospy
import time

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
#from lunabot_control.scripts.pid_controller import VelocityPIDController
#from lunabot_control.scripts.clamp_output import clamp_output

from lunabot_behavior.excavate import ExcavationController
from lunabot_behavior.deposition import DepositionManager
from lunabot_behavior.linear_actuators import LinearActuatorManager
from lunabot_behavior.alignment import AlignmentController
from lunabot_behavior.traversal import TraversalManager
import lunabot_behavior.zones as zones
from lunabot_behavior.homing import HomingController

from std_msgs.msg import Int8, Bool, Int32
import sys
import math

class ExdepController:
    """
    Controls the excavation-deposition autonomous cycle.
    """

    def apriltag_pose_callback(self, msg: PoseStamped):
        self.apriltag_pose_in_odom = msg
        
        # Find the mining/berm zones in the odom frame
        self.mining_zone: zones.Zone = zones.find_mining_zone(self.apriltag_pose_in_odom, self.is_sim)
        self.berm_zone: zones.Zone = zones.find_berm_zone(self.apriltag_pose_in_odom, self.is_sim)


    def __init__(self, excavation_publisher: rospy.Publisher = None, 
                       linear_actuator_publisher: rospy.Publisher = None, 
                       cmd_vel_publisher: rospy.Publisher = None, 
                       deposition_publisher: rospy.Publisher = None,
                       traversal_manager: TraversalManager = None,
                       led_publisher: rospy.Publisher = None,
                       apriltag_enabled_publisher: rospy.Publisher = None):
        """
        If passed a publisher, then it is assumed a node is already running, and the publisher is shared.
        Else, initialize this node to run on its own.
        """
        if (excavation_publisher is None or linear_actuator_publisher is None or
           cmd_vel_publisher is None or deposition_publisher is None):
            rospy.init_node('exdep_node')

            self.excavation_publisher: rospy.Publisher = rospy.Publisher("/excavate", Int32, queue_size=1, latch=True)
            self.lin_act_publisher: rospy.Publisher = rospy.Publisher("/lin_act", Int32, queue_size=1, latch=True)
            self.deposition_publisher: rospy.Publisher = rospy.Publisher("/deposition", Int32, queue_size=1, latch=True)
            self.cmd_vel_publisher: rospy.Publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1, latch=True)
            self.traversal_manager = TraversalManager()
            self.led_publisher = rospy.Publisher("/led_color", Int32, queue_size=1, latch=True)
            self.apriltag_enabled_publisher = rospy.Publisher("/apriltag/enabled", Bool, queue_size=1, latch=True)

        else:
            self.excavation_publisher = excavation_publisher
            self.lin_act_publisher = linear_actuator_publisher
            self.deposition_publisher = deposition_publisher
            self.cmd_vel_publisher = cmd_vel_publisher
            self.traversal_manager = traversal_manager
            self.led_publisher = led_publisher
            self.apriltag_enabled_publisher = apriltag_enabled_publisher


        self.excavation = ExcavationController(self.excavation_publisher, self.lin_act_publisher, self.cmd_vel_publisher, self.deposition_publisher)
        self.deposition = DepositionManager(self.deposition_publisher)
        self.linear_actuators = LinearActuatorManager(self.lin_act_publisher)
        self.alignment = AlignmentController(self.cmd_vel_publisher)
        self.homing = HomingController(self.cmd_vel_publisher)
        self.is_sim = rospy.get_param("/is_sim")

        self.apriltag_pose_in_odom = None
        self.mining_zone: zones.Zone = None
        self.berm_zone: zones.Zone = None

        apriltag_topic = rospy.get_param("/apriltag_topic")
    
        rospy.Subscriber(apriltag_topic, PoseStamped, self.apriltag_pose_callback)

        self.rate = rospy.Rate(10) #hz

    def set_color(self, new_color: Int32):
        self.led_publisher.publish(new_color)


    def exdep_loop(self):
        """
        The main loop for the excavation-deposition cycle. Assumes it starts at the mining zone, ready to dig.
        """
        rospy.sleep(0.1)

        UCF_BOTTOM = False # are we on the bottom UCF map?

        # wait until we have our apriltag position
        while (self.apriltag_pose_in_odom is None):
            rospy.sleep(0.1)

        self.cycle_count = 0
        while (not rospy.is_shutdown()):

            ####################
            # Excavation
            ####################
            
            self.set_color(4) # White for excavation

            # we start in the mining zone, hopefully at a good mining location
            # only plunge first two cycles to ensure full autonomy points

            # Note- this is changed back to normal until plunging can be stopped correctly
            if self.cycle_count < 2:
                self.excavation.excavate()
            else:
                self.excavation.excavate()

            self.cycle_count += 1

            ####################
            # Raise Linear Actuators
            ####################

            # spin excavation while raising
            exc_msg = Int32()
            exc_msg.data = 1500
            self.excavation_publisher.publish(exc_msg)

            # raise the linear actuators out of the way
            self.linear_actuators.raise_linear_actuators(True) 

            # stop spinning excavation
            exc_msg = Int32()
            exc_msg.data = 0
            self.excavation_publisher.publish(exc_msg)

            ####################
            # Traversal to Berm
            ####################

            # pick berm area goal
            berm_goal = PoseStamped()
            berm_goal.pose.position.x = self.berm_zone.middle[0]
            berm_goal.pose.position.y = self.berm_zone.middle[1]
            berm_goal.header.stamp = rospy.Time.now()
            berm_goal.header.frame_id = "odom"

            # aim for one meter above the berm (reversed for UCF bottom map)
            if UCF_BOTTOM:
                offset = zones.calc_offset(0, -1, self.apriltag_pose_in_odom, self.is_sim) 
            else:
                offset = zones.calc_offset(0, 1, self.apriltag_pose_in_odom, self.is_sim)

            berm_goal.pose.position.x += offset[0]
            berm_goal.pose.position.y += offset[1]

            # move to the berm area
            rospy.loginfo("Behavior: Moving to berm area")
            self.set_color(2) # Green for traversal
            self.traversal_manager.traverse_to_goal(berm_goal, drive_backwards=True)

            # once we've arrived, stop.
            cmd_vel = Twist()
            cmd_vel.linear.x = 0
            cmd_vel.angular.z = 0
            self.cmd_vel_publisher.publish(cmd_vel)

            ####################
            # Alignment 
            ####################

            self.set_color(7) # Purple for deposition autonomy
            
            self.homing.initalize()

            rospy.loginfo("Homing: Aligning to Berm Apriltag")
            self.homing.home()
            
            rospy.loginfo("Homing: Moving to Berm Apriltag")
            self.homing.approach()

            rospy.sleep(1)

            ####################
            # Deposition
            ####################

            rospy.loginfo("Behavior: Depositing")
            self.deposition.deposit()

            # get out of mining area for 3 sec
            cmd_vel.linear.x = 0.3
            cmd_vel.angular.z = 0
            self.cmd_vel_publisher.publish(cmd_vel)

            rospy.sleep(3)

            # stop
            cmd_vel.linear.x = 0
            cmd_vel.angular.z = 0
            self.cmd_vel_publisher.publish(cmd_vel)

            rospy.sleep(0.3)

            ####################
            # Traversal to Mining Zone
            ####################

            mining_goal = PoseStamped()
            mining_goal.pose.position.x = self.mining_zone.middle[0]
            mining_goal.pose.position.y = self.mining_zone.middle[1]
            mining_goal.header.stamp = rospy.Time.now()
            mining_goal.header.frame_id = "odom"

            random_offset = self.mining_zone.random_offset()
            offset = zones.calc_offset(random_offset[0], random_offset[1], self.apriltag_pose_in_odom, self.is_sim)  # TODO: maybe shift mining location each time
            mining_goal.pose.position.x += offset[0]
            mining_goal.pose.position.y += offset[1]

            # move to the mining area
            rospy.loginfo("Behavior: Moving to mining area")
            self.set_color(2) # Green for traversal
            self.traversal_manager.traverse_to_goal(mining_goal, drive_backwards=False)

            # once we've arrived, stop.
            cmd_vel = Twist()
            cmd_vel.linear.x = 0
            cmd_vel.angular.z = 0
            self.cmd_vel_publisher.publish(cmd_vel)

if __name__ == "__main__":
    exdep_controller = ExdepController()
    exdep_controller.exdep_loop()
    rospy.spin()
