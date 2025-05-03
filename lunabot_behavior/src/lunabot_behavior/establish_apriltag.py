import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Int8, Int32


class EstablishAprilTag:
    def apriltag_pose_callback(self, msg: PoseStamped):
        self.apriltag_pose_in_odom = msg
    
    def __init__(self):
        rospy.init_node('establish_tag_node')
        self.apriltag_pose_in_odom: PoseStamped = None
        self.apriltag_enabled_publisher = rospy.Publisher("/apriltag/enabled", Bool, queue_size=1, latch=True)
        self.apriltag_pose_publisher = rospy.Publisher("/apriltag_pose", PoseStamped, queue_size=1, latch=True)
        self.rate = rospy.Rate(1)
        apriltag_topic = rospy.get_param("/apriltag_topic")
        rospy.Subscriber(apriltag_topic, PoseStamped, self.apriltag_pose_callback)
        self.APRILTAG_AVERAGING_TIME = 5.0
        
    def establish_tag(self):
        rospy.loginfo("Behavior: April Tag Detected")
        rospy.loginfo("Behavior: Collecting average april tag pose")
        
        self.apriltag_enabled_publisher.publish(True) # enable apriltag detection

        rospy.sleep(1)
        
        if self.apriltag_pose_in_odom == None:
            rospy.logerr("Behavior: Could not find apriltag")
            return

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
        
if __name__ == "__main__":
    apriltag = EstablishAprilTag()
    apriltag.establish_tag()
    rospy.spin()
