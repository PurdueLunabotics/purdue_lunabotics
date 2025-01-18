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
import lunabot_behavior.zones as zones

from std_msgs.msg import Int8, Bool
import sys
import math

class ExdepController:
    """
    Controls the excavation-deposition autonomous cycle.
    """

    def apriltag_pose_callback(self, msg: PoseStamped):
        self.apriltag_pose_in_odom = msg
        
        print("got apriltag")
        # Find the mining/berm zones in the odom frame
        self.mining_zone: zones.Zone = zones.find_mining_zone(self.apriltag_pose_in_odom, self.is_sim)
        self.berm_zone: zones.Zone = zones.find_berm_zone(self.apriltag_pose_in_odom, self.is_sim)

    def odom_callback(self, msg: Odometry):
        self.robot_odom = msg

    def __init__(self, excavation_publisher: rospy.Publisher = None, 
                       linear_actuator_publisher: rospy.Publisher = None, 
                       cmd_vel_publisher: rospy.Publisher = None, 
                       deposition_publisher: rospy.Publisher = None,
                       traversal_publisher: rospy.Publisher = None,
                       goal_publisher: rospy.Publisher = None):
        """
        If passed a publisher, then it is assumed a node is already running, and the publisher is shared.
        Else, initialize this node to run on its own.
        """
        if (excavation_publisher is None or linear_actuator_publisher is None or
           cmd_vel_publisher is None or deposition_publisher is None or
           traversal_publisher is None or goal_publisher is None):
            rospy.init_node('exdep_node')

            self.excavation_publisher: rospy.Publisher = rospy.Publisher("/excavate", Int8, queue_size=1, latch=True)
            self.lin_act_publisher: rospy.Publisher = rospy.Publisher("/lin_act", Int8, queue_size=1, latch=True)
            self.deposition_publisher: rospy.Publisher = rospy.Publisher("/deposition", Int8, queue_size=1, latch=True)
            self.cmd_vel_publisher: rospy.Publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1, latch=True)
            self.traversal_publisher: rospy.Publisher = rospy.Publisher("/behavior/traversal_enabled", Bool, queue_size=1, latch=True)
            self.goal_publisher: rospy.Publisher = rospy.Publisher("/goal", PoseStamped, queue_size=1, latch=True)
        else:
            self.excavation_publisher = excavation_publisher
            self.lin_act_publisher = linear_actuator_publisher
            self.deposition_publisher = deposition_publisher
            self.cmd_vel_publisher = cmd_vel_publisher
            self.traversal_publisher = traversal_publisher
            self.goal_publisher = goal_publisher

        self.excavation = ExcavationController(self.excavation_publisher, self.lin_act_publisher, self.cmd_vel_publisher)
        self.deposition = DepositionManager(self.deposition_publisher)
        self.linear_actuators = LinearActuatorManager(self.lin_act_publisher)
        self.alignment = AlignmentController(self.cmd_vel_publisher)

        self.is_sim = rospy.get_param("/is_sim")

        self.apriltag_pose_in_odom = None
        self.mining_zone: zones.Zone = None
        self.berm_zone: zones.Zone = None

        apriltag_topic = rospy.get_param("/apriltag_topic")
    
        rospy.Subscriber(apriltag_topic, PoseStamped, self.apriltag_pose_callback)

        self.robot_odom = Odometry()
        odom_topic = rospy.get_param("/odom_topic")
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback)

        self.rate = rospy.Rate(10) #hz

        
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

    def exdep_loop(self):
        """
        The main loop for the excavation-deposition cycle. Assumes it starts at the mining zone, ready to dig.
        """
        rospy.sleep(0.1)

        print("wait")
        # wait until we have our apriltag position
        while (self.apriltag_pose_in_odom is None):
            rospy.sleep(0.1)

        print("done")
        while (not rospy.is_shutdown):

            # we start in the mining zone, hopefully at a good mining location
            self.excavation.excavate()

            # raise the linear actuators out of the way
            self.linear_actuators.raise_linear_actuators()

            # pick berm area goal
            berm_goal = PoseStamped()
            berm_goal.pose.position.x = self.berm_zone.middle[0]
            berm_goal.pose.position.y = self.berm_zone.middle[1]
            berm_goal.header.stamp = rospy.Time.now()
            berm_goal.header.frame_id = "odom"

            offset = zones.calc_offset(0, 1, self.apriltag_pose_in_odom, self.is_sim)  # TODO; maybe shift deposit location each time
            berm_goal.pose.position.x += offset[0]
            berm_goal.pose.position.y += offset[1]

            self.goal_publisher.publish(berm_goal)

            # move to the berm area
            rospy.loginfo("Behavior: Moving to berm area")
            traversal_message = Bool()
            traversal_message.data = True
            self.traversal_publisher.publish(traversal_message)

            while not self.is_close_to_goal(berm_goal):
                self.rate.sleep()

            traversal_message.data = False
            self.traversal_publisher.publish(traversal_message)

            # once we've arrived, stop.
            cmd_vel = Twist()
            cmd_vel.linear.x = 0
            cmd_vel.angular.z = 0
            self.cmd_vel_publisher.publish(cmd_vel)

            # align to the berm
            self.alignment.align_to_angle(math.pi / 2)

            # Todo: maybe add some kind of approach logic?

            self.deposition.deposit()

            mining_goal = PoseStamped()
            mining_goal.pose.position.x = self.mining_zone.middle[0]
            mining_goal.pose.position.y = self.mining_zone.middle[1]
            mining_goal.header.stamp = rospy.Time.now()
            mining_goal.header.frame_id = "odom"

            offset = zones.calc_offset(0, 0, self.apriltag_pose_in_odom, self.is_sim)  # TODO: maybe shift mining location each time
            mining_goal.pose.position.x += offset[0]
            mining_goal.pose.position.y += offset[1]

            self.goal_publisher.publish(mining_goal)

            # move to the mining area
            rospy.loginfo("Behavior: Moving to mining area")
            traversal_message.data = True
            self.traversal_publisher.publish(traversal_message)

            while not self.is_close_to_goal(mining_goal):
                self.rate.sleep()
            
            traversal_message.data = False
            self.traversal_publisher.publish(traversal_message)

            # once we've arrived, stop.
            cmd_vel = Twist()
            cmd_vel.linear.x = 0
            cmd_vel.angular.z = 0
            self.cmd_vel_publisher.publish(cmd_vel)

if __name__ == "__main__":
    exdep_controller = ExdepController()
    exdep_controller.exdep_loop()
    rospy.spin()