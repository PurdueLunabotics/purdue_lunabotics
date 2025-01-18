import rospy
from geometry_msgs.msg import Twist, PoseStamped
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
import tf2_ros
import tf2_geometry_msgs

class AprilTag:
    
    def apriltag_callback(self, msg: AprilTagDetectionArray):
        #print(len(msg.detections))
        if len(msg.detections) > 0:
            #xprint(self.found_apriltag)
            detection = msg.detections[0]
            frameid = detection.pose.header.frame_id

            if (not self.is_sim and frameid != "d455_back_color_optical_frame"): # only use front camera
                print("wrong camera")
                #return
            else:
                rospy.loginfo("Behavior: found apriltag")
                self.apriltag_pub.publish(self.convert_to_odom_frame(detection))
        
    
    def __init__(self):
        
        self.is_sim = rospy.get_param("/is_sim")
        if self.is_sim:
            cam_topic = "/d435_backward/color/tag_detections"
        else:
            cam_topic = "/d455_back/camera/color/tag_detections"
                
                
        self.apriltag_pub = rospy.Publisher("/apriltag_pose", PoseStamped, queue_size=10)
        rospy.Subscriber(cam_topic, AprilTagDetectionArray, self.apriltag_callback)
        
        self.rate = rospy.Rate(10)  # 10hz
        
        
    def loop(self):
        rospy.init_node("apriltag_node")    
        
        while not rospy.is_shutdown():
            self.rate.sleep()    
            
        
    def convert_to_odom_frame(self, apriltag_detection: AprilTagDetection):
        """
        Convert the first apriltag detection to the odom frame
        """
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        source_frame = self.apriltag_detections.header.frame_id
        target_frame = "odom"

        pose = tf2_geometry_msgs.PoseStamped()
        pose.header = self.apriltag_detections.header
        pose.pose = apriltag_detection.pose.pose.pose

        # Set the time to 0 to get the latest available transform
        pose.header.stamp = rospy.Time(0)

        pose_in_odom = tf_buffer.transform(pose, target_frame, rospy.Duration(5.0))

        return pose_in_odom


if __name__ == "__main__":
    detection = AprilTag()
    detection.loop()
    