import rospy
from geometry_msgs.msg import Twist

from apriltag_ros.msg import AprilTagDetectionArray

def main():
    global foundTag
    foundTag = False

	rospy.init_node("init_mapping_node")

	vel_msg = Twist()
    rate = rospy.Rate(20)

	vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
	#vel_pub.publish(vel_msg)

	apriltags = rospy.Subscriber(
            "/d455_front/camera/color/tag_detections",
            AprilTagDetectionArray,
            apritag_cb,
    )

    start = rospy.get_time()
    
    while True:
        if not rospy.is_shutdown():
            break
        if foundTag:
            vel_pub.linear.x = 0
	        vel_pub.angular.z = 0
            vel_pub.publish(vel_msg)
            return True
        if rospy.get_time() - start >= 45: #no tags for too long
            break

        #keep turning and searching for april_tags
        vel_pub.angular.z = 0.2
        vel_pub.publish(vel_msg)
        rate.sleep()
        
    vel_pub.linear.x = 0
	vel_pub.angular.z = 0
    vel_pub.publish(vel_msg)
	return False

def apritag_cb(self, msg):
    if len(msg.detections) != 0:
        foundTag = True
    else:
        foundTag = False
