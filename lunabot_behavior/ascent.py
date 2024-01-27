import rospy
from lunabot_msgs.msg import RobotEffort, RobotSensors

def main():
	rospy.init_node("ascent_behavior_node")

	rospy.Rate(10)
	
	effort_pub = rospy.Publisher("/effort", RobotEffort, queue_size=1)
	sensor_sub = rospy.Subscriber("/sensors", RobotSensors, self._robot_state_cb)

	while not rospy.is_shutdown(): #and current sensors not at limit
		pass
		# ascend

	return True