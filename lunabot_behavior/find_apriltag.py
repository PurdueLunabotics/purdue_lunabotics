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
        self.apriltag_detections = msg;
        if len(msg.detections) > 0:
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
        
        #TODO change to parameter
        # Real /d455_front/camera/color/tag_detections
        # Sim /d435_backward/color/tag_detections
        rospy.Subscriber("/d435_backward/color/tag_detections", AprilTagDetectionArray, self.apriltag_callback)

        self.rate = rospy.Rate(10)  # 10hz

    def find_apriltag(self):
        """
        Spin in a circle (in the starting zone) until an apriltag is found. Then stop and return.
        """
        
        time.sleep(0.1)

        start = rospy.get_time()

        velocity_message = Twist()

        while True:
            if self.found_apriltag:
                velocity_message.linear.x = 0
                velocity_message.angular.z = 0

                self.velocity_publisher.publish(velocity_message)

                return self.apriltag_detections.detections[0]
                
            if rospy.get_time() - start >= 45: #no tags for too long
                break # exit and return false

            # keep turning and searching for april_tags
            velocity_message.linear.x = 0
            velocity_message.angular.z = 0.261799 # Around 15 degrees
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

        # This requests a transform time half a second in the future from when the apriltag was found
        # This is needed because the transform listener is slow, and doesn't have the correct transform from when the apriltag is found exactly
        pose.header.stamp = rospy.Time(pose.header.stamp.to_sec() + 0.4, 0)

        try:
            pose_in_odom = tf_buffer.transform(pose, target_frame, rospy.Duration(2.0))
        except:
            # If this doesn't work (occasionally it doesn't,) look even farther in the future (although this may be less accurate)
            try:
                pose.header.stamp = rospy.Time(pose.header.stamp.to_sec() + 0.8, 0)
                pose_in_odom = tf_buffer.transform(pose, target_frame, rospy.Duration(4.0))
            except:
                raise Exception("Failed to transform apriltag pose to odom frame")

        return pose_in_odom
        

    
if __name__ == '__main__':
    find_apriltag = FindAprilTag()
    find_apriltag.find_apriltag()

    
