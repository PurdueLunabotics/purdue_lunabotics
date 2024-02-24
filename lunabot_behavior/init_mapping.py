import rospy
from geometry_msgs.msg import Twist

def main(vel_pub, foundTag):
    vel_pub = Twist()

    while True:
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