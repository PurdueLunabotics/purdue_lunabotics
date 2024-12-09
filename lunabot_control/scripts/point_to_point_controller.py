import rospy
import math
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from pid_controller import PIDController
from tf.transformations import euler_from_quaternion


class PointToPoint:
    def __init__(self, linear_kP=0.1, linear_kI=0, linear_kD=0, angular_kP=0.1, angular_kI=0, angular_kD=0, angular_tolerance=0.1, pos_tolerance=0.05, max_linear_vel=0.5, max_angular_vel=0.5):
        """
        Initializes the PointToPoint controller with the given PID values and tolerances.
        
        :param linear_kP: Proportional gain for linear velocity
        :param linear_kI: Integral gain for linear velocity
        :param linear_kD: Derivative gain for linear velocity
        :param angular_kP: Proportional gain for angular velocity
        :param angular_kI: Integral gain for angular velocity
        :param angular_kD: Derivative gain for angular velocity
        :param angular_tolerance: Tolerance for angular velocity
        :param pos_tolerance: Tolerance for position
        :param max_linear_vel: Maximum linear velocity
        :param max_angular_vel: Maximum angular velocity
        """
        self.LINEAR_P = linear_kP
        self.LINEAR_I = linear_kI
        self.LINEAR_D = linear_kD

        self.ANGULAR_P = angular_kP
        self.ANGULAR_I = angular_kI
        self.ANGULAR_D = angular_kD

        self.FREQUENCY = 60 # Hz

        self.ANGULAR_TOLERANCE = angular_tolerance # radians
        self.POSITION_TOLERANCE = pos_tolerance # meters

        self.linear_controller = PIDController(self.LINEAR_P, self.LINEAR_I, self.LINEAR_D)
        self.angular_controller = PIDController(self.ANGULAR_P, self.ANGULAR_I, self.ANGULAR_D)

        self.angular_velocity = 0
        self.linear_velocity = 0

        self.robot_pose = (0, 0, 0) # (x, y, theta)
        self.robot_velocity = [0, 0] # [linear, angular]
        self.last_pos = (0, 0, 0)

        self.target = (0, 0)
        self.path = [self.robot_pose[:2]]

        self.dt = 1 / self.FREQUENCY
        self.prev_time = rospy.Time.now().to_sec()

        self.MAX_LINEAR_VELOCITY = max_linear_vel
        self.MAX_ANGULAR_VELOCITY = max_angular_vel

        cmd_vel_topic = rospy.get_param('/cmd_vel_topic')
        self.cmd_vel_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    def read_path_callback(self, msg: Path):
        path = [self.robot_pose[:2]]
        for point in msg.poses:
            path.append(tuple((point.pose.position.x, point.pose.position.y)))

        if path != self.path: # only update path if it's different
            self.path = path
            self.initialize_target_point(self.robot_pose)
            
    def odom_callback(self, msg: Odometry):
        # self.robot_velocity = [msg.twist.twist.linear, msg.twist.twist.angular]
        angles = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.robot_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, angles[2])
        self.robot_velocity = [math.sqrt((self.robot_pose[0] - self.last_pos[0]) ** 2 + (self.robot_pose[1] - self.last_pos[1]) ** 2)
                               / (self.dt), ((self.robot_pose[2] - self.last_pos[2])/(self.dt))]

    def initialize_target_point(self, robot_pose: tuple) -> None:
        for self.i in range(self.path - 1):
            if robot_pose[0] > self.path[self.i][0] and robot_pose[1] > self.path[self.i][1] and robot_pose[0] < self.path[self.i + 1][0] and robot_pose[1] < self.path[self.i + 1][1]:
                self.target = self.path[self.i + 1] # return next point if robot is in path
                return
            
        self.i = 0
        self.target = self.path[self.i]

    def follow_path(self):
        """
        Follows the path by turning to the target point and then moving towards it. Called periodically. 
        """
        # turn to point
        angle_target = math.atan2(self.target[1] - self.robot_pose[1], self.target[0] - self.robot_pose[0]) + self.robot_pose[2]
        if (abs(self.robot_pose[2] - angle_target) > self.ANGULAR_TOLERANCE):
            self.linear_velocity = 0
            self.angular_velocity = self.angular_controller.calculate(self.robot_pose[2], self.dt, setpoint=angle_target)
        else: # translate to point
            dist_to_target = math.sqrt((self.target[0] - self.robot_pose[0]) ** 2 + (self.target[1] - self.robot_pose[1]) ** 2)
            if (dist_to_target < self.POSITION_TOLERANCE):
                if (self.i + 1 < len(self.path)):
                    self.i += 1
                    self.target = self.path[self.i]
            else:
                self.angular_velocity = 0
                self.linear_velocity = self.linear_controller.calculate(dist_to_target, self.dt, setpoint=0)

    def publish_telemetry(self):
        vel = Twist()
        vel.linear.x = self.clamp(self.linear_velocity, -self.MAX_LINEAR_VELOCITY, self.MAX_LINEAR_VELOCITY)
        vel.angular.z = self.clamp(self.angular_velocity, -self.MAX_ANGULAR_VELOCITY, self.MAX_ANGULAR_VELOCITY)
        self.cmd_vel_publisher.publish(vel)

    def clamp(self, value, min_value, max_value) -> float:
        return max(min(value, max_value), min_value)

    def run_node(self):
        rospy.init_node('point_to_point_controller')

        path_topic = rospy.get_param('/nav/global_path_topic')
        rospy.Subscriber(path_topic, Path, self.read_path_callback)

        odom_topic = rospy.get_param('/odom_topic')
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback)

        rate = rospy.Rate(self.FREQUENCY)

        while not rospy.is_shutdown():
            self.follow_path()
            self.publish_telemetry()
            self.dt = rospy.Time.now().to_sec() - self.prev_time
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('point_to_point_node')
    ptp = PointToPoint()
    ptp.run_node()