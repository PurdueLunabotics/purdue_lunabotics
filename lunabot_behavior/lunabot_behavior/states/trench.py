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

class Drive(State):
    def __init__(self, target_distance: float, backwards: bool, name: str = "drive") -> None:
        self.target_distance = target_distance
        self.backwards = backwards
        self.name = name

    def setup(self, manager: Node):
        self.logger = manager.get_logger()
        self.cmd_vel_pub = manager.create_publisher(Twist, "cmd_vel", 10)
        self.odom_sub = manager.create_subscription(PoseStamped, "position", self.odom_cb, 1)
        self.linear_pid = ParameterizedPIDController(f"{self.name}.linear", manager, kp=1.0, max_output=0.1)
        self.odom = None
        self.position = (0, 0)
        # while self.odom == None:
        #     rclpy.spin_once(manager)
        #     time.sleep(0.25)

    def odom_cb(self, pose: PoseStamped):
        self.odom = pose
        self.position = (pose.pose.position.x, pose.pose.position.y)

    def start(self):
        self.starting_pos = self.position

    def periodic(self) -> None | Events:
        distance = np.sqrt((self.position[0] - self.starting_pos[0])**2 + (self.position[1] - self.starting_pos[1])**2)
        if abs(distance - self.target_distance) <= 0.1:
            return Events.SUCCESS

        output = Twist()
        output.linear.x = self.linear_pid.calculate(distance, 0.1, self.target_distance) * (1 if not self.backwards else -1)
        self.cmd_vel_pub.publish(output)

    def exit(self):
        self.cmd_vel_pub.publish(Twist())

class Trench(Drive):
    def __init__(self) -> None:
        super().__init__(0.5, False, "trench")

    def setup(self, manager: Node):
        super().setup(manager)

        self.state_manager = manager

        self.excavation_pub = manager.create_publisher(Int32, "excavate", 10)
        self.dep_pub = manager.create_publisher(Int32, "deposition", 10)

        self.EXCAVATION_SPEED = 1500 # rpm
        self.DEPOSITION_SPEED = 200 # rpm

    def start(self):
        super().start()

    def periodic(self) -> None | Events:
        self.excavation_pub.publish(Int32(data = self.EXCAVATION_SPEED))
        self.dep_pub.publish(Int32(data = self.DEPOSITION_SPEED))

        return super().periodic()
    
    def exit(self):
        self.excavation_pub.publish(Int32(data = 0))
        self.dep_pub.publish(Int32(data = 0))
        super().exit()
