import rospy
from geometry_msgs.msg import Twist, PoseStamped
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
import tf2_ros
import tf2_geometry_msgs

import interrupts

import time

'''
Called init_mapping in the concept of operations, this is the first behavior state- spinning until an apriltag is found
and then reporting back on the estimated locations of the mining zone and berm zone
'''
class FindAprilTag:

    def apriltag_callback(self, msg: AprilTagDetectionArray):
        self.apriltag_detections = msg
        #print(len(msg.detections))
        if len(msg.detections) > 0:
            #xprint(self.found_apriltag)
            detection = msg.detections[0]
            frameid = detection.pose.header.frame_id

            if (not self.is_sim and frameid != "d455_back_color_optical_frame"): # only use front camera
                print("wrong camera")
                #return

            self.found_apriltag = True


    def __init__(self, velocity_publisher: rospy.Publisher = None):
        """
        If passed a publisher, then it is assumed a node is already running, and the publisher is shared.
        Else, initialize this node to run on its own.
        """

        if velocity_publisher is None:
            self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1, latch=True)
            rospy.init_node('find_apriltag_node')
        else:
            self.velocity_publisher = velocity_publisher

        self.found_apriltag = False
        self.apriltag_detections = AprilTagDetectionArray()

        self.is_sim = rospy.get_param("/is_sim")
        if self.is_sim:
            cam_topic = "/d435_backward/color/tag_detections"
        else:
            cam_topic = "/d455_back/camera/color/tag_detections"
        #print(cam_topic)
        rospy.Subscriber(cam_topic, AprilTagDetectionArray, self.apriltag_callback)

        self.rate = rospy.Rate(10)  # 10hz

    def find_apriltag(self):
        """
        Spin in a circle (in the starting zone) until an apriltag is found. Then stop and return.
        """

        # TODO look for the right apriltag bundle
        
        time.sleep(0.1)

        start = rospy.get_time()

        velocity_message = Twist()

        while True:
            if self.found_apriltag:
                rospy.loginfo("Behavior: found apriltag")
                velocity_message.linear.x = 0
                velocity_message.angular.z = 0

                for i in range(10):
                    self.velocity_publisher.publish(velocity_message)
                    rospy.sleep(0.25)

                return self.apriltag_detections.detections[0]
                
            if rospy.get_time() - start >= 45: #no tags for too long
                break # exit and return false

            # keep turning and searching for april_tags
            velocity_message.linear.x = 0
            velocity_message.angular.z = -0.5 #0.261799 # Around 15 degrees
            self.velocity_publisher.publish(velocity_message)

            if interrupts.check_for_interrupts() != interrupts.Errors.FINE:
                return "Error"

            self.rate.sleep()
            
        velocity_message.linear.x = 0
        velocity_message.angular.z = 0
        self.velocity_publisher.publish(velocity_message)

        return None
    
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
    
    def spin(self):

        TIME = 10 # seconds
        SPEED = 1 # radians/s

        velocity_message = Twist()
        velocity_message.linear.x = 0

        start_time = rospy.get_time()

        velocity_message.angular.z = SPEED

        while rospy.get_time() - start_time < TIME:
            self.velocity_publisher.publish(velocity_message)
            self.rate.sleep()

        velocity_message.angular.z = 0

        self.velocity_publisher.publish(velocity_message)

        

    
if __name__ == '__main__':
    find_apriltag = FindAprilTag()
    #find_apriltag.spin()
    find_apriltag.find_apriltag()

    

