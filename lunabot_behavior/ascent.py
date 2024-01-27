import rospy
from lunabot_msgs.msg import RobotEffort, RobotSensors

def robot_state_cb(self, msg):
	global drive_left_curr, drive_right_curr, dep_curr
	drive_left_curr = msg.drive_left_curr
	drive_right_curr = msg.drive_right_curr
	dep_curr = msg.dep_curr


def main():
	rospy.init_node("ascent_behavior_node")

	rate = rospy.Rate(10)
	
	effort_pub = rospy.Publisher("/effort", RobotEffort, queue_size=1)
	sensor_sub = rospy.Subscriber("/sensors", RobotSensors, robot_state_cb)

	while not rospy.is_shutdown(): #and current sensors not at limit
		# ascend
		rate.sleep()

	return True