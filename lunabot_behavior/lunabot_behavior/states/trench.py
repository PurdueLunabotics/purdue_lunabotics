from rclpy.node import Node
from rclpy.duration import Duration

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

LINEAR_TOLERANCE = 0.1

class Drive(State):
    def __init__(self, target_distance: float, backwards: bool, speed: float, timeout: float=0.0) -> None:
        self.timeout = timeout
        self.target_distance = target_distance
        self.backwards = backwards
        self.speed = speed
        self.stalled = False

    def setup(self, manager: Node):
        self.logger = manager.get_logger()
        self.cmd_vel_pub = manager.create_publisher(Twist, "cmd_vel", 10)
        self.odom_sub = manager.create_subscription(PoseStamped, "position", self.odom_cb, 1)
        self.odom = None
        self.position = (0, 0)
        self.elapsed = 0
        
        self.manager = manager
        # while self.odom == None:
        #     rclpy.spin_once(manager)
        #     time.sleep(0.25)

    def odom_cb(self, pose: PoseStamped):
        self.odom = pose
        self.position = (pose.pose.position.x, pose.pose.position.y)

    def start(self):
        self.start_time = self.manager.get_clock().now().nanoseconds / 1e9
        self.starting_pos = self.position
        if self.stalled:
            self.timeout -= self.elapsed
            self.stalled = False

    def periodic(self) -> None | Events:
        distance = np.sqrt((self.position[0] - self.starting_pos[0])**2 + (self.position[1] - self.starting_pos[1])**2)
        
        time = self.manager.get_clock().now().nanoseconds / 1e9 # current time in seconds
        is_timeout = self.timeout > 0.0 and time - self.start_time > self.timeout
        if abs(distance - self.target_distance) <= LINEAR_TOLERANCE or is_timeout:
            return Events.SUCCESS

        output = Twist()
        output.linear.x = -self.speed if self.backwards else self.speed
        self.cmd_vel_pub.publish(output)

    def exit(self, event):
        self.cmd_vel_pub.publish(Twist())
        self.timeout += self.elapsed
        if event is Events.STALL:
            self.stalled = True
            self.elapsed += self.manager.get_clock().now().nanoseconds / 1e9 - self.start_time
        else:
            self.elapsed = 0

class Trench(Drive):
    def __init__(self) -> None:
        super().__init__(0.5, False, speed=0.03, timeout=30.0)

    def setup(self, manager: Node):
        super().setup(manager)

        self.state_manager = manager

        self.excavation_pub = manager.create_publisher(Int32, "excavate", 10)
        self.dep_pub = manager.create_publisher(Int32, "deposition", 10)

        self.EXCAVATION_SPEED = 2000 # rpm
        self.DEPOSITION_SPEED = 200 # rpm

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
