import rospy

from geometry_msgs.msg import Twist


"""
A class that controls the behavior for if the robot becomes 'stuck'
Rocks back and fowards in order to become unstuck- (TODO - this might displace us too much)
"""
class Escape:
	NUM_ITERATIONS = 6  # how many times to rock back and forth
	ESCAPE_SPEED = 1    # m/s

	def __init__(self, velocity_publisher: rospy.Publisher = None):
		if velocity_publisher is None:
			self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1, latch=True)
			rospy.init_node('escape_node')
		else:
			self.velocity_publisher = velocity_publisher

	def drive_in_line(self, speed: float):
		velocity_message = Twist()
		velocity_message.linear.x = speed

		self.velocity_publisher.publish(velocity_message)
		
	def unstickRobot(self):
		# rock backwards and forwards
		for i in range(self.NUM_ITERATIONS):

			rospy.sleep((i+1)/2)
			self.drive_in_line(-self.ESCAPE_SPEED)

			rospy.sleep((i+1)/2)
			self.drive_in_line(self.ESCAPE_SPEED)

		rospy.sleep(2)

		# stop
		velocity_message = Twist()
		velocity_message.linear.x = 0
		velocity_message.angular.z = 0
		self.velocity_publisher.publish(velocity_message)

		return True
	
if __name__ == "__main__":
	escape = Escape()
	escape.unstickRobot()