import rospy

from lunabot_msgs.msg import RobotSensors
from std_msgs.msg import Int8

import time
import interrupts

class Ascent:
	'''
	This is a transition state used to raise the linear actuators to the maximum height.
	'''

	def sensors_callback(self, msg: RobotSensors):
		self.robot_sensors = msg
	
	def __init__(self, lin_act_publisher: rospy.Publisher = None):
		"""
		If passed a publisher, then it is assumed a node is already running, and the publisher is shared.
		Else, initialize this node to run on its own.
		"""

		if lin_act_publisher is None:
			self.lin_act_publisher = rospy.Publisher("/lin_act", Int8, queue_size=1, latch=True)
			rospy.init_node('ascent_node')
		else:
			self.lin_act_publisher = lin_act_publisher

		self.robot_sensors = RobotSensors()

		rospy.Subscriber("/sensors", RobotSensors, self.sensors_callback)

		self.rate = rospy.Rate(10)  # 10hz

		self.ACTUATOR_CURRENT_THRESHOLD = 0.01 #TODO adjust as needed

		self.RAISING_TIME = 15

		self.LIN_ACT_POWER = 110

		self.is_sim = rospy.get_param("is_sim")

	def raise_linear_actuators(self):
		"""
		Raise linear actuators to the max. height by turning them on until the current received is 0.
		"""

		# don't run if in sim
		if (self.is_sim):
			rospy.loginfo("Ascent: would raise actuators")
			time.sleep(2)
			return True

		time.sleep(0.1)

		lin_act_msg = Int8()
		lin_act_msg.data = self.LIN_ACT_POWER

		start_time = rospy.get_time()

		while (rospy.get_time() - start_time < self.RAISING_TIME):
			self.lin_act_publisher.publish(lin_act_msg)

			#TODO check for new sensor message / values
			if (self.robot_sensors.act_right_curr - 0) < self.ACTUATOR_CURRENT_THRESHOLD:
				# If the current is within EPSILON of 0, then end
				break

			if (interrupts.check_for_interrupts() != interrupts.Errors.FINE):
				return False

			self.rate.sleep()

		lin_act_msg.data = 0
		self.lin_act_publisher.publish(lin_act_msg)
		
		return True
	
if __name__ == "__main__":
	ascent = Ascent()
	ascent.raise_linear_actuators()