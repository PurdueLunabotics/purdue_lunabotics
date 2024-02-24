import rospy
from geometry_msgs.msg import Twist
from apriltag_ros.msg import AprilTagDetectionArray

'''
Called init_mapping in the concept of operations, this is the first behavior state- spinning until an apriltag is found
and then reporting back on the estimated locations of the mining zone and berm zone
'''
class FindAprilTag:

    def apriltag_callback(self, msg: AprilTagDetectionArray):
        if len(msg.detections) > 0:
            self.found_apriltag = True

    def __init__(self, velocity_publisher: rospy.Publisher = None):

        if velocity_publisher is None:
            self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1, latch=True)
            rospy.init_node('find_apriltag_node')
        else:
            self.velocity_publisher = velocity_publisher

        self.found_apriltag = False
        
        rospy.Subscriber("/d455_front/camera/color/tag_detections", AprilTagDetectionArray, self.apriltag_callback)

        self.rate = rospy.Rate(10)  # 10hz

    def find_apriltag(self):
        start = rospy.get_time()

        velocity_message = Twist()

        while True:
            if self.found_apriltag:
                velocity_message.linear.x = 0
                velocity_message.angular.z = 0

                self.velocity_publisher.publish(velocity_message)

                return True
                
            if rospy.get_time() - start >= 45: #no tags for too long
                break # exit and return false

            # keep turning and searching for april_tags
            velocity_message.linear.x = 0
            velocity_message.angular.z = 0.261799 # Around 15 degrees
            self.velocity_publisher.publish(velocity_message)

            self.rate.sleep()
            
        velocity_message.linear.x = 0
        velocity_message.angular.z = 0
        self.velocity_publisher.publish(velocity_message)

        return False
    
if __name__ == '__main__':
    find_apriltag = FindAprilTag()
    find_apriltag.find_apriltag()
    rospy.spin()