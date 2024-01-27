import rospy
from lunabot_msgs.msg import RobotEffort, RobotSensors
from geometry_msgs.msg import Twist

def robot_state_cb(self, msg):
	global drive_left_curr, drive_right_curr, dep_curr
	drive_left_curr = msg.drive_left_curr
	drive_right_curr = msg.drive_right_curr
	dep_curr = msg.dep_curr

def main():
	rospy.init_node("escape_behavior_node")

	vel_msg = Twist()

	vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

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

def line(vel, power):
	vel.linear.x = power
	vel.angular.z = 0