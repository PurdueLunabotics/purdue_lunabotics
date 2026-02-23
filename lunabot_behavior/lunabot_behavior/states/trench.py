from rclpy.node import Node
from rclpy.duration import Duration

from lunabot_behavior.state import State, Events


from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from lunabot_msgs.msg import RobotSensors

# TODO: what happens if the node stalls? does the time reset? also should we do distance not time based?

class Trench(State):
    def setup(self, manager: Node):
        self.state_manager = manager
        self.excavation_pub = manager.create_publisher(Int32, "excavate", 10)
        self.linact_pub = manager.create_publisher(Int32, "lin_act", 10)
        self.cmdvel_pub = manager.create_publisher(Twist, "cmd_vel", 10)
        self.dep_pub = manager.create_publisher(Int32, "deposition", 10)

        # Constants TODO: update these
        self.TRENCHING_TIME = 30  # seconds
        # speed to run drivetrain during trenching (m/s)
        self.TRENCHING_SPEED = 0.01 # lil slow - exc stalled at 0.02, try 0.015 next
        self.EXCAVATION_SPEED = 1500 # rpm
        self.DEPOSITION_SPEED = 1000 # rpm
        self.LIN_ACT_CURR_THRESHOLD = 0.1  # Amps
        self.LIN_ACT_MAX_POWER = 110

    def start(self):
        self.start_time = self.state_manager.get_clock().now()

    def periodic(self) -> None | Events:
        self.linact_pub.publish(Int32(data = self.LIN_ACT_MAX_POWER)) # TODO: why?
        self.excavation_pub.publish(Int32(data = self.EXCAVATION_SPEED))
        self.dep_pub.publish(Int32(data = self.DEPOSITION_SPEED))

        cmd = Twist()
        cmd.linear.x = self.TRENCHING_SPEED
        cmd.angular.z = 0.0
        self.cmdvel_pub.publish(cmd)

        elapsed = self.state_manager.get_clock().now() - self.start_time
        if elapsed > Duration(seconds=self.TRENCHING_TIME):
            return Events.SUCCESS
        
        return None 
    
    def exit(self):
        self.excavation_pub.publish(Int32(data = 0))
        self.dep_pub.publish(Int32(data = 0))
        self.linact_pub.publish(Int32(data = 0))
        self.cmdvel_pub.publish(Twist())
