from lunabot_msgs.msg import RobotEffort
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32 # Import the type we need

class EffortFactory(Node): # Effort factory is a subclass of Node
    def __init__(self):
        super().__init__("effort_factory") # Initialize the node with the name 

        self.left_drive = 0 # set a default value for the left drive
        self.left_drive_sub = self.create_subscription(Int32, "left_drive", self.left_drive_cb, 10) # Create the subscription

        self.right_drive = 0 # set a default value for the left drive
        self.right_drive_sub = self.create_subscription(Int32, "right_drive", self.right_drive_cb, 10) # Create the subscription

        self.lin_act = 0 # set a default value for the left drive
        self.lin_act_sub = self.create_subscription(Int32, "lin_act", self.lin_act_cb, 10) # Create the subscription

        self.effort_pub = self.create_publisher(RobotEffort, "effort", 10)

        self.timer = self.create_timer(0.1, self.loop)

    def left_drive_cb(self, left_drive: Int32): # This is called every time something publishes to the topic
        self.left_drive = left_drive.data # Int32 is not actually a number, so we must access the data inside

    def right_drive_cb(self, right_drive: Int32): # This is called every time something publishes to the topic
        self.right_drive = right_drive.data # Int32 is not actually a number, so we must access the data inside

    def lin_act_cb(self, lin_act: Int32): # This is called every time something publishes to the topic
        self.lin_act = lin_act.data # Int32 is not actually a number, so we must access the data inside

    def loop(self):
        effort = RobotEffort()
        effort.left_drive = self.left_drive
        effort.right_drive = self.right_drive
        effort.lin_act = self.lin_act
        self.effort_pub.publish(effort)
        
def main():
    rclpy.init()
    effort_factory = EffortFactory()
    rclpy.spin(effort_factory) # Tells the runtime to wait and process messages and timers for this node until the node is stopped
    rclpy.shutdown()
