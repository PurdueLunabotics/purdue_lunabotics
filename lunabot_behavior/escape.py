import rospy

from geometry_msgs.msg import Twist

def main():
	vel_msg = Twist()

	escape_speed = 5

	#rock backwards and forwards
	for i in range(6):
		rospy.sleep((i+1)/2)
		line(vel_msg, -escape_speed)
		vel_pub.publish(vel_msg)

		rospy.sleep((i+1)/2)
		line(vel_msg, escape_speed)
		vel_pub.publish(vel_msg)

	rospy.sleep(2)
	line(vel_msg, 0)
	vel_pub.publish(vel_msg)

	return True

def line(vel, power):
	vel.linear.x = power
	vel.angular.z = 0