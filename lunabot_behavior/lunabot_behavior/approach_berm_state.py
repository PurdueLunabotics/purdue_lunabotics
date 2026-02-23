from state import State, Events
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped

class ApproachBerm(State):    
  def setup(self, manager:Node):
    self.cmd_vel_publisher = manager.create_publisher(Twist, "cmd_vel", 10)
    manager.create_subscription(PoseStamped, "position", self.odom_cb, 1)
    self.manager = manager
    self.robot_pose = (None, None)
    self.linear_speed = 0.1 #m/s
    self.target_y = 1.5
  
  def odom_cb(self, msg:PoseStamped):
    self.robot_pose = (
        msg.pose.position.x,
        msg.pose.position.y,
    )
    
  def start(self):
    pass
  
  def periodic(self):
    if self.robot_pose[0] == None:
      return None
    linear_error = self.target_y - self.robot_pose[1]
    output = Twist()
    if linear_error < 0:
      return Events.SUCCESS
    else:
      output.linear.x = -self.linear_speed
    self.cmd_vel_publisher.publish(output)
    return None
  
  def exit(self):
    self.cmd_vel_publisher.publish(Twist())
    
