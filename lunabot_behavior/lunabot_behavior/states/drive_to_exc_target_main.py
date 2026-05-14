from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.parameter import Parameter

from lunabot_behavior.state import State, Events
from lunabot_behavior.states.trench import Drive

from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Twist
from lunabot_msgs.msg import RobotSensors, Linkup
from lunabot_control.pid_controller import ParameterizedPIDController
from tf_transformations import euler_from_quaternion
import numpy as np
import time
import rclpy

class DriveToMainExcTarget(Drive):
    def __init__(self) -> None:
        super().__init__(0, False, speed=0.2, timeout=30.0, tolerance=0.05)

    def setup(self, manager: Node):
        super().setup(manager)

        self.state_manager = manager

        self.state_manager.create_subscription(Linkup, "/linkup_pos", self.linkup_cb, 10)

        self.target_dist_computed = False

    def linkup_cb(self, msg: Linkup):
        if self.position != (None, None) and not self.target_dist_computed:
            l_x = msg.main_target.x
            l_y = msg.main_target.y
            self.target_distance = np.sqrt((l_x - self.position[0]) ** 2 + (l_y - self.position[1]) ** 2)
            self.target_dist_computed = True

    def start(self):
        self.target_dist_computed = False
        super().start()

    def periodic(self):
        if self.target_dist_computed:
            return super().periodic()