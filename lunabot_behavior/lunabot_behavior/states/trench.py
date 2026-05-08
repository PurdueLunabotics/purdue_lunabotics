from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.parameter import Parameter

from lunabot_behavior.state import State, Events

from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Twist
from lunabot_msgs.msg import RobotSensors
from lunabot_control.pid_controller import ParameterizedPIDController
from tf_transformations import euler_from_quaternion
import numpy as np
import time
import rclpy

# TODO: what happens if the node stalls? does the time reset? also should we do distance not time based?

LINEAR_TOLERANCE = 0.05
TRENCH_DISTANCE = 0.5

class Drive(State):
    def __init__(self, target_distance: float, backwards: bool, speed: float, timeout: float=0.0, tolerance: float=LINEAR_TOLERANCE, use_pid: bool=False) -> None:
        self.timeout = timeout
        self.target_distance = target_distance
        self.backwards = backwards
        self.speed = speed
        self.stalled = False
        self.tolerance = tolerance
        self.use_pid = False

    def setup(self, manager: Node):
        self.logger = manager.get_logger()
        self.cmd_vel_pub = manager.create_publisher(Twist, "cmd_vel", 10)
        self.odom_sub = manager.create_subscription(PoseStamped, "position", self.odom_cb, 1)
        self.odom = None
        self.position = (None, None)
        self.elapsed = 0
        
        self.manager = manager

        kp = 0.1
        ki = 0.001
        kd = 0
        self.pid = ParameterizedPIDController("drive", self.manager, kp, ki, kd, max_output=self.speed)

    def odom_cb(self, pose: PoseStamped):
        self.odom = pose
        self.position = (pose.pose.position.x, pose.pose.position.y)

    def start(self):
        self.start_time = self.manager.get_clock().now().nanoseconds / 1e9
        self.position = (None, None)
        self.starting_pos = self.position
        if self.stalled:
            self.timeout -= self.elapsed
            self.stalled = False

        self.last_position = (None, None)

    def periodic(self) -> None | Events:
        if (self.position != (None, None) and self.starting_pos == (None, None)): # position got updated by odom
            self.starting_pos = self.position

        if (self.position != (None, None)):
            distance = np.sqrt((self.position[0] - self.starting_pos[0])**2 + (self.position[1] - self.starting_pos[1])**2)
            self.manager.get_logger().info(f"[DRIVE] Dist: {distance:.2f} | Target Dist: {self.target_distance:.2f}")

            time = self.manager.get_clock().now().nanoseconds / 1e9 # current time in seconds
            is_timeout = self.timeout > 0.0 and time - self.start_time > self.timeout
            if abs(distance - self.target_distance) <= self.tolerance or is_timeout:
                return Events.SUCCESS

            output = Twist()

            if self.use_pid:
                speed = -self.pid.calculate(distance, 0.1, 0)
            else:
                speed = self.speed

            output.linear.x = -speed if self.backwards else speed
            
            self.cmd_vel_pub.publish(output)
            return None

    def exit(self, event):
        self.cmd_vel_pub.publish(Twist())
        self.timeout += self.elapsed
        if event is Events.STALL:
            self.stalled = True
            self.elapsed += self.manager.get_clock().now().nanoseconds / 1e9 - self.start_time
        else:
            self.elapsed = 0

        self.last_position = self.position

class Trench(Drive):
    def __init__(self) -> None:
        super().__init__(TRENCH_DISTANCE, False, speed=0.03, timeout=30.0, tolerance=0.05)

    def setup(self, manager: Node):
        super().setup(manager)

        self.state_manager = manager

        self.excavation_pub = manager.create_publisher(Int32, "excavate", 10)
        self.dep_pub = manager.create_publisher(Int32, "deposition", 10)

        self.manager.declare_parameter("trenching_dist", TRENCH_DISTANCE)

        self.EXCAVATION_SPEED = 2000 # rpm
        self.DEPOSITION_SPEED = 400 # rpm # TODO: tune

    def start(self):
        super().start()

    def periodic(self) -> None | Events:
        self.excavation_pub.publish(Int32(data = self.EXCAVATION_SPEED))
        self.dep_pub.publish(Int32(data = self.DEPOSITION_SPEED))

        return super().periodic()
    
    def exit(self, event):
        self.excavation_pub.publish(Int32(data = 0))
        self.dep_pub.publish(Int32(data = 0))
        super().exit(event)
