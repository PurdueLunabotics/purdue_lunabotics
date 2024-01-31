import rospy
from lunabot_msgs.msg import RobotEffort, RobotSensors
from geometry_msgs.msg import Twist

from std_msgs.msg import Bool

def robot_state_cb(self, msg):
	global left_curr, right_curr, left_vel, right_vel
	left_curr = msg.drive_left_curr
	right_curr = msg.drive_right_curr
	left_vel = msg.drive_left_vel
	right_vel = msg.drive_right_vel

def eff_cb(self, msg):
	global cmd_left, cmd_right
	cmd_left = msg.drive_left
	cmd_right = msg.drive_right

def stuck():
	rospy.init_node("stuck_node")
	stuck = Bool()

	stuck.data = False

	s = rospy.Publisher("/stuck", Bool, queue_size=1)
	rospy.Subscriber("/effort", RobotEffort, eff_cb)
	rospy.Subscriber("/sensors", RobotSensors, robot_state_cb)

	s.publish(stuck)

	stuck_time_lock = False
	stuck_time = rospy.get_time()

	stuck_min_time = 3.0 #seconds

	r = rospy.Rate(20)

	while not rospy.is_shutdown():
		if stuck_time - rospy.get_time() >= stuck_min_time:
			stuck.data = True
			s.publish(stuck)
		else:
			stuck.data = False
			s.publish(stuck)

		#check if trying to go somewhere
		if cmd_left >= 0.05 or cmd_right >= 0.05:
			# check if not going anywhere
			if left_vel <= 0.05 and right_vel <= 0.05
				if not stuck_time_lock:
					stuck_time_lock = True
					stuck_time = rospy.get_time()
				continue

		stuck_time_lock = False
		rate.sleep()



	stuck.data = False