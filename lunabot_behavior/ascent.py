import rospy

from lunabot_msgs.msg import RobotEffort, RobotSensors

import time

EPSILON = 0.01 #TODO adjust as needed

class Ascent:

	def sensors_callback(self, msg: RobotSensors):
		self.robot_sensors = msg
	
	def __init__(self, effort_publisher: rospy.Publisher = None):

		if effort_publisher is None:
			self.effort_publisher = rospy.Publisher("/effort", RobotEffort, queue_size=1, latch=True)
			rospy.init_node('ascent_node')
		else:
			self.effort_publisher = effort_publisher

		self.robot_sensors = RobotSensors()

		rospy.Subscriber("/sensors", RobotSensors, self.sensors_callback)

	def raiseLinearActuators(self):
		time.sleep(0.1)

		effort_message = RobotEffort()
		effort_message.lin_act = 127 #TODO check max

		while True:
			self.effort_publisher.publish(effort_message)

			#TODO check for new sensor message / values
			if (self.robot_sensors.act_right_currrent - 0) < EPSILON:
				# If the current is within EPSILON of 0, then end
				break

		effort_message.lin_act = 0
		self.effort_publisher.publish(effort_message)
		
		return