#!/usr/bin/env python3

import numpy as np
import math

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Pose, Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion

class HomingController:
    """
    This class aligns the robot to the apriltag for deposition
    """

    # How far should the robot be from the apriltag pos / angle
    linear_setpoint = 0.8
    angular_setpoint = 0

    alignment_threshold = 0.1 # in rad, how close to align before stopping
    
    BERM_TAG_IDS = [2,6]

    # Linear, Angular
    KP = np.array([0.0, 2.0])
    KI = np.array([0.0, 0.0])
    KD = np.array([0.0, 0.0])

    linear_limits = np.array([-0.25, 0.25]) #m/s
    angular_limits = np.array([-0.2, 0.2]) #rad/s

    def __init__(self, cmd_vel_publisher: rospy.Publisher = None):
        """
        If passed a publisher, then it is assumed a node is already running, and the publisher is shared.
        Else, initialize this node to run on its own.
        """

        if cmd_vel_publisher is None:
            self.cmd_vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1, latch=True)
            rospy.init_node("homing_controller_node")
        else:
            self.cmd_vel_publisher = cmd_vel_publisher


        self.is_sim = rospy.get_param("/is_sim")

        self.cam_mode = "back"  # front, back, or sim

        # Decide on which camera to use
        if self.is_sim:
            cam_topic = "/d435_backward/color/tag_detections"
            self.cam_mode = "sim"
            rospy.loginfo("Homing Controller: Sim")
        else:
            cam_topic = "/d455_back/camera/color/tag_detections"
            self.cam_mode = "back"
            rospy.loginfo("Homing Controller: Using back cam")

        self.apriltag_subscriber = rospy.Subscriber(cam_topic,  AprilTagDetectionArray, self.apritag_callback)

        self.cmd_vel = Twist()

        self.berm_apriltag_position: Pose = None
        self.berm_apriltag_header: Header = None

        odom_topic = rospy.get_param("/odom_topic")

        self.odom: Odometry = None
        self.odom_subscriber = rospy.Subscriber(odom_topic, Odometry, self.odom_callback)

        self.apriltag_publisher = rospy.Publisher("/apriltag_visual", PoseStamped, queue_size=1)

        self.prev_error = np.zeros(2)
        self.curr_error = np.zeros(2)
        self.error_total = np.zeros(2)

        self.rate = rospy.Rate(20)

    def initalize(self):
        self.berm_apriltag_position: Pose = None
        self.berm_apriltag_header: Header = None

    def apritag_callback(self, msg: AprilTagDetectionArray):
        if len(msg.detections) != 0:
            if (msg.detections[0].id[0] in self.BERM_TAG_IDS):
                self.berm_apriltag_position = msg.detections[0].pose.pose.pose
                self.berm_apriltag_header = msg.detections[0].pose.header

                tag_pose_stamped = PoseStamped()
                tag_pose_stamped.header = self.berm_apriltag_header
                tag_pose_stamped.pose = self.berm_apriltag_position
                self.apriltag_publisher.publish(tag_pose_stamped)
            else:
                pass
                #rospy.logerr("wrong tag: " + str(msg.detections[0].id[0]))
        else:
            self.berm_apriltag_position = None

    def odom_callback(self, msg: Odometry):
        self.odom = msg

    def spin_until_apriltag(self):

        self.cmd_vel.angular.z = 0.2 # around 45 degrees per second

        if self.cam_mode == "sim":
            self.cmd_vel.angular.z = 0.2 # around 22.5 degrees per second this needs to be slower
        
        if self.berm_apriltag_position is None: #if there is an apriltag don't turn
            while self.berm_apriltag_position is None:
                self.cmd_vel_publisher.publish(self.cmd_vel)
                self.rate.sleep()
            rospy.sleep(0.5) #end more centered

        self.stop()

    def home(self):
        """
        Align the robot to the apriltag
        """

        self.spin_until_apriltag()
        rospy.sleep(1)

        while (True):

            tf_buffer = tf2_ros.Buffer()
            tf_listener = tf2_ros.TransformListener(tf_buffer)

            target_frame = "odom"

            pose = tf2_geometry_msgs.PoseStamped()
            pose.header = self.berm_apriltag_header
            pose.pose = self.berm_apriltag_position

            # Set the time to 0 to get the latest available transform
            pose.header.stamp = rospy.Time(0)
            try:
                pose_in_odom = tf_buffer.transform(pose, target_frame, rospy.Duration(1.0))
            except AttributeError:
                continue

            euler_angles = euler_from_quaternion([pose_in_odom.pose.orientation.x, pose_in_odom.pose.orientation.y, pose_in_odom.pose.orientation.z, pose_in_odom.pose.orientation.w])
            if (self.cam_mode == "back"):
                apriltag_yaw = euler_angles[2] + np.pi / 2
            elif (self.cam_mode == "sim"):
                apriltag_yaw = euler_angles[2] - np.pi / 2 # In sim, adjust the apriltag to point 'out' of the apriltag, by 90 deg

            target_yaw = math.atan2((pose_in_odom.pose.position.y - self.odom.pose.pose.position.y), (pose_in_odom.pose.position.x - self.odom.pose.pose.position.x))
            

            robot_yaw = euler_from_quaternion([self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w])[2]
            robot_yaw = robot_yaw + np.pi

            angular_error = target_yaw - robot_yaw
            angular_error = (angular_error + np.pi) % (2 * np.pi) - np.pi

            # print(str(target_yaw) + "  " + str(robot_yaw))
            print("Error:", angular_error)
            
            
            # Stopping point
            if abs(angular_error) < self.alignment_threshold:
                self.stop()
                rospy.loginfo("Homing Controller: Done Homing")
                break

            self.curr_error = np.array([0, angular_error])

            # Computing PID control from error
            control = self.curr_error * self.KP
            
            # Set current error to previous error (for D)
            self.prev_error = self.curr_error

            # Publish the control (and constrain it)
            cmd_vel_message = Twist()
            cmd_vel_message.linear.x = 0
            cmd_vel_message.angular.z = np.clip(control[1]*2, self.angular_limits[0], self.angular_limits[1])

            self.cmd_vel_publisher.publish(cmd_vel_message)

            self.rate.sleep()
        rospy.sleep(2) #remove later
        return True
    def approach(self):
        """
        Approach the apriltag. After you have homed/ are facing the apriltag, drive in straight line
        """

        if self.is_sim:
            DIST_THRESHOLD = 1.3 # meters, how close to the apriltag to stop
        else:
            DIST_THRESHOLD = 1.0 # real robot camera is on the back edge of robot, so stop closer
        APPROACH_SPEED = -0.2 # m/s

        last_apriltag_position = self.berm_apriltag_position

        missed_apriltag_counter = 0

        while (True):
            
            if (last_apriltag_position != self.berm_apriltag_position and self.berm_apriltag_position is not None):
                last_apriltag_position = self.berm_apriltag_position
            
            if (self.berm_apriltag_position is None):
                self.berm_apriltag_position = last_apriltag_position
                missed_apriltag_counter += 1

                if (missed_apriltag_counter >= 7):
                    self.stop()
                    rospy.loginfo("Homing: Early end")
                    return True

            print("apriltag", self.berm_apriltag_position.position)
            
            #print(distance_notsquared)
            if (self.berm_apriltag_position is not None and self.berm_apriltag_position.position.z < DIST_THRESHOLD):
                self.stop()
                return True
            elif (self.berm_apriltag_position is None):
                missed_apriltag_counter += 1
                continue

            self.cmd_vel.linear.x = APPROACH_SPEED
            self.cmd_vel.angular.z = 0

            self.cmd_vel_publisher.publish(self.cmd_vel)

            self.rate.sleep()

    def stop(self):
        self.cmd_vel.linear.x = 0
        self.cmd_vel.angular.z = 0
        self.cmd_vel_publisher.publish(self.cmd_vel)

if __name__ == "__main__":
    homing_controller = HomingController()
    homing_controller.home()
