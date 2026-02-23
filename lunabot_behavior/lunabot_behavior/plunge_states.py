from rclpy.node import Node
from rclpy.duration import Duration

from state import State
from state import Events

from std_msgs.msg import Int32
from lunabot_msgs.msg import RobotSensors

class Plunge(State):
    def setup(self, manager: Node):
        self.excavation_pub = manager.create_publisher(Int32, "excavate", 10)
        self.linact_pub = manager.create_publisher(Int32, "linact", 10)
        self.manager = manager

        self.sensor_sub = manager.create_subscription(Int32, "sensors", self.sensor_callback, 10)
        self.sensors = None

        # Constants (in meters)  TODO: update these 
        self.PLUNGE_TIME = 30  # seconds
        self.MIN_TIME = 2  # seconds
        self.EXCAVATION_SPEED = 1500 # rpm
        self.LIN_ACT_MAX_POWER = 127 # -127 - 127
        self.LIN_ACT_CURR_THRESHOLD = 0.1  # Amps; TODO find value

    def sensor_callback(self, sensors: RobotSensors):
        self.sensors = sensors

    def start(self):
        self.start_time = self.manager.get_clock().now()

    def periodic(self) -> None | Events:
        if self.sensors is None:
            return None
        
        self.excavation_pub.publish(1000)
        self.linact_pub.publish(self.LIN_ACT_MAX_POWER)
        
        elapsed = self.manager.get_clock().now() - self.start_time
        if elapsed > Duration(seconds=self.PLUNGE_TIME):
                return Events.SUCCESS
        
        
        if abs(self.sensors.act_right_curr) < self.LIN_ACT_CURR_THRESHOLD and elapsed > Duration(seconds=self.MIN_TIME):
            return Events.SUCCESS
        return None
            
    
    def exit(self):
        self.excavation_pub.publish(Int32(0))
        self.linact_pub.publish(Int32(0))
 
