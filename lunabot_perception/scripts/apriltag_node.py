#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
from visualization_msgs.msg import Marker
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion

from lunabot_behavior.zones import Zone, find_mining_zone, find_berm_zone

class ApriltagNode:
    
    def apriltag_callback(self, msg: AprilTagDetectionArray):
        if len(msg.detections) > 0:

            self.apriltag_detection_array = msg

    
    def __init__(self):

        rospy.init_node("apriltag_node")

        self.apriltag_detection_array: AprilTagDetectionArray = None
        self.apriltag_pose_in_odom: PoseStamped = None

        self.apriltag_publisher = rospy.Publisher("/apriltag_pose", PoseStamped, queue_size=10, latch=True)
        self.mining_zone_visual_publisher = rospy.Publisher("/mining_zone", Marker, queue_size=10, latch=True)
        self.berm_zone_visual_publisher = rospy.Publisher("/berm_zone", Marker, queue_size=10, latch=True)
        
        self.is_sim = rospy.get_param("/is_sim")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        if self.is_sim:
            cam_topics = ["/d435_backward/color/tag_detections", "/d435_forward/color/tag_detections"]
        else:
            cam_topics = ["/d455_back/camera/color/tag_detections", "/d455_front/camera/color/tag_detections",
                         "/d435_left/camera/color/tag_detections", "/d435_right/camera/color/tag_detections"]
        
        subscribers = []
        for topic in cam_topics:
            subscriber = rospy.Subscriber(topic, AprilTagDetectionArray, self.apriltag_callback)
            subscribers.append(subscriber)
        
        self.frequency = 10

        self.apriltag_update_needed: bool = False
                
        
    def loop(self):
        
        rate = rospy.Rate(self.frequency)
        
        while not rospy.is_shutdown():
            if (self.apriltag_detection_array is not None):
                self.translate_and_publish_apriltag()
            rate.sleep()   

    def translate_and_publish_apriltag(self):
        # Todo- identify the apriltag bundle we want
        detection = self.apriltag_detection_array.detections[0]
        
        self.apriltag_pose_in_odom = self.convert_to_odom_frame(detection)
        self.apriltag_publisher.publish(self.apriltag_pose_in_odom)

        mining_zone: Zone = find_mining_zone(self.apriltag_pose_in_odom, self.is_sim)
        berm_zone: Zone = find_berm_zone(self.apriltag_pose_in_odom, self.is_sim)

        mining_zone.visualize_zone(self.mining_zone_visual_publisher, id=1, color=(1,0,0,1))
        berm_zone.visualize_zone(self.berm_zone_visual_publisher, id=2, color=(0,1,1,1))
            
        
    def convert_to_odom_frame(self, apriltag_detection: AprilTagDetection):
        """
        Convert the first apriltag detection to the odom frame
        """

        # tf_buffer = tf2_ros.Buffer()
        # tf_listener = tf2_ros.TransformListener(tf_buffer)

        target_frame = "odom"

        pose = tf2_geometry_msgs.PoseStamped()
        pose.header = apriltag_detection.pose.header
        pose.pose = apriltag_detection.pose.pose.pose

        # Set the time to 0 to get the latest available transform
        # pose.header.stamp = rospy.Time(0)

        pose_in_odom = self.tf_buffer.transform(pose, target_frame, rospy.Duration(5.0))

        return pose_in_odom


if __name__ == "__main__":
    apriltag_node = ApriltagNode()
    apriltag_node.loop()
    