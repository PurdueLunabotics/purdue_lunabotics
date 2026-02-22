from rclpy.node import Node
from rclpy.duration import Duration

from state import State
from state import Events

from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from lunabot_msgs.msg import RobotSensors

class trench_state(State):
    def setup(self, manager: Node):
        self.state_manager = manager
        self.excavation_pub = manager.create_publisher(Int32, "excavate", 10)
        self.linact_pub = manager.create_publisher(Int32, "linact", 10)
        self.cmdvel_pub = manager.create_publisher(Twist, "cmd_vel", 10)

        self.sensors_sub = manager.create_subscription(RobotSensors, "sensors", self.sensors_callback, 10)
        
        self.sensors = None

        # Constants TODO: update these
        self.TRENCHING_TIME = 30  # seconds
        # speed to run drivetrain during trenching (m/s)
        self.TRENCHING_SPEED = 0.01 # lil slow - exc stalled at 0.02, try 0.015 next
        self.EXCAVATION_SPEED = 1500 # rpm
        self.LIN_ACT_CURR_THRESHOLD = 0.1  # Amps; TODO find value
        self.MAX_LIN_ACT_VEL = 0.00688405797  # In meters/s, the speed of the linear actuators at the max power (from experiment - 19 cm / 27.6 seconds)
        self.LIN_ACT_MAX_POWER = 110

    def start(self):
        self.start_time = self.state_manager.get_clock().now()

    def periodic(self) -> None | Events:
        if self.sensors is None:
            return None
        
        self.linact_pub.publish(self.LIN_ACT_MAX_POWER)
        self.excavation_pub.publish(self.EXCAVATION_SPEED)

        cmd = Twist()
        cmd.linear.x = self.TRENCHING_SPEED
        cmd.angular.z = 0.0
        self.cmdvel_pub.publish(cmd)

        elapsed = self.state_manager.get_clock().now() - self.start_time
        if elapsed > Duration(seconds=self.TRENCHING_TIME):
            return Events.SUCCESS
        
        return None 
    
    def exit(self):
        self.excavation_pub.publish(0)
        self.linact_pub.publish(0)
        self.cmdvel_pub.publish(Twist())
