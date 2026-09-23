import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class EffortFactory(Node):
    def __init__(self):
        super().__init__("effort_factory")

        self.left_drive = 0
        self.left_drive_sub = self.create_subscription(Int32, "left_drive",
            self.left_drive_cb, 10)

        self.right_drive = 0
        self.right_drive_sub = self.create_subscription(Int32, "right_drive",
            self.right_drive_cb, 10)

    def left_drive_cb(self, left_drive: Int32):
        self.left_drive = left_drive.data
    
    def right_drive_cb(self, right_drive: Int32):
        self.right_drive = right_drive.data
    
    self.effort_pub = self.create_publisher(RobotEffort, "effort", 10)
    self.timer = self.create_timer(0.1, self.loop) # 10 hertz

    def loop(self):
        effort = RobotEffort() # create an empty message
        effort.left_drive = self.left_drive
        effort.right_drive = self.right_drive
        self.effort_pub.publish(effort)
        
def main():
    rclpy.init()
    effort_factory = EffortFactory()
    rclpy.spin(effort_factory)
    rclpy.shutdown()