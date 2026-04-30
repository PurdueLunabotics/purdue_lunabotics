#!/usr/bin/python3
import rclpy
from rclpy.node import Node

from lunabot_msgs.msg import RobotEffort, RobotStall, RobotSensors, Event
import threading

class StallDetector(Node):
    def __init__(self, **kwargs):
        super().__init__('stall_detector_node', **kwargs)
        rclpy.get_global_executor().add_node(self)

        effort_topic = "effort"
        self.create_subscription(RobotEffort, effort_topic, self.effort_callback, 1)
        sensors_topic = "sensors"
        self.create_subscription(RobotSensors, sensors_topic, self.sensors_callback, 1)
        
        self.declare_parameter("~waitTime",2)
        
        self.stall_publisher = self.create_publisher(RobotStall, "stalled", 10)
        self.event_publisher = self.create_publisher(Event, "events", 10)
        self.effort = None
        self.stall = RobotStall()
        self.stallCounter = {'left':0,
                             'right':0,
                             'exc':0}
        
    def effort_callback(self, msg: RobotEffort):
        self.effort = msg

    def sensors_callback(self, msg: RobotSensors):
        time = self.get_parameter("~waitTime").get_parameter_value().integer_value
        # first is left, second is right, third is exc
        if self.effort == None:
            return
        
        if abs(msg.drive_left_vel) < 10 and abs(self.effort.left_drive) > 100:
            self.stallCounter['left'] += 1
        else:
            self.stallCounter['left'] = 0
        if abs(msg.drive_right_vel) < 10 and abs(self.effort.right_drive) > 100:
            self.stallCounter['right'] += 1
        else:
            self.stallCounter['right'] = 0
        if abs(msg.exc_vel) < 10 and abs(self.effort.excavate) > 100:
            self.stallCounter['exc'] += 1
        else:
            self.stallCounter['exc'] = 0
        
        if (self.stallCounter['left'] > time):
            self.stall.left_stall = True
        if (self.stallCounter['right'] > time):
            self.stall.right_stall = True
        if (self.stallCounter['exc'] > time):
            self.stall.exc_stall = True

        if (any(count > time for count in self.stallCounter.values())):
            self.event_publisher.publish(Event(data = Event.STALL))
        
        self.stall_publisher.publish(self.stall)
        

def main(args=None):
    rclpy.init(args=args)

    node = StallDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
