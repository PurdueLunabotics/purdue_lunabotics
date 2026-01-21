#!/usr/bin/python3
import rclpy
from rclpy.node import Node

from lunabot_msgs.msg import RobotEffort
from lunabot_msgs.msg import RobotStall
from lunabot_msgs.msg import RobotSensors
import threading

class StallDetector(Node):
    def __init__(self, **kwargs):
        super().__init__('stall_detector_node', **kwargs)
        rclpy.get_global_executor().add_node(self)

        effort_topic = "/effort"
        self.create_subscription(effort_topic, RobotEffort, self.effort_callback, 1)
        sensors_topic = "/sensors"
        self.create_subscription(sensors_topic, RobotSensors, self.sensors_callback, 1)
        
        self.stall_publisher = self.create_publisher(RobotStall, "/stalled", 10)
        self.stall = RobotStall
        
    def effort_callback(self, msg: RobotEffort):
        self.effort = msg

    def sensors_callback(self, msg: RobotSensors):
        # first is left, second is right, third is exc
        
        if abs(msg.drive_left_vel) < 10 and abs(self.effort.left_drive) > 0:
            self.stall.left_stall = True
        if abs(msg.drive_right_vel) < 10 and abs(self.effort.right_drive) > 0:
            self.stall.right_stall = True
        if abs(msg.excavate_vel) < 10 and abs(self.effort.excavate) > 0:
            self.stall.exc_stall = True
        
        
        self.stall_publisher.publish(self.stall)
        
        
def spin_in_background():
    executor = rclpy.get_global_executor()
    try:
        executor.spin()
    except Exception:
        pass
    
if __name__ == "__main__":
    rclpy.init()
    t = threading.Thread(target=spin_in_background)
    t.start()
    stall_detector = StallDetector()
    stall_detector.run_node()