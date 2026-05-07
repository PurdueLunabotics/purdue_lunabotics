from rclpy.node import Node
from rclpy.duration import Duration

from lunabot_behavior.state import State, Events

from std_msgs.msg import Int32
from lunabot_msgs.msg import RobotSensors

class Raise(State):
    def setup(self, manager: Node):
        self.excavation_pub = manager.create_publisher(Int32, "excavate", 10)
        self.linact_pub = manager.create_publisher(Int32, "lin_act", 10)
        self.manager = manager

        self.sensor_sub = manager.create_subscription(RobotSensors, "sensors", self.sensor_callback, 10)
        self.sensors = None

        # Constants  TODO: update these 
        self.RAISE_TIME = 10  # seconds
        self.MIN_TIME = 2
        self.EXCAVATION_SPEED = 2000 # rpm
        self.LIN_ACT_MAX_POWER = 127 # -127 - 127
        self.LIN_ACT_CURR_THRESHOLD = 0.1  # Amps; TODO find value

    def sensor_callback(self, sensors: RobotSensors):
        self.sensors = sensors

    def start(self):
        self.start_time = self.manager.get_clock().now()

    def periodic(self) -> None | Events:
        if self.sensors is None:
            return None
        
        self.excavation_pub.publish(Int32(data = self.EXCAVATION_SPEED))
        self.linact_pub.publish(Int32(data = self.LIN_ACT_MAX_POWER))
        
        elapsed = self.manager.get_clock().now() - self.start_time
        if elapsed > Duration(seconds=self.RAISE_TIME):
                return Events.SUCCESS
        
        
        if abs(self.sensors.act_right_curr) < self.LIN_ACT_CURR_THRESHOLD and elapsed > Duration(seconds=self.MIN_TIME):
            return Events.SUCCESS
        return None
            
    
    def exit(self, event):
        self.excavation_pub.publish(Int32(data = 0))
        self.linact_pub.publish(Int32(data = 0))
 
