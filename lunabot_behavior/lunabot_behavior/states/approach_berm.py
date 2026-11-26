from lunabot_behavior.zones import ZoneMeasurements
from lunabot_behavior.state import State, Events
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped

from lunabot_control.pid_controller import ParameterizedPIDController

OFFSET_FROM_CENTER = 0.2

class ApproachBerm(State):    
  def __init__(self, backwards = False):
    self.backwards = backwards

  def setup(self, manager:Node):
    self.cmd_vel_publisher = manager.create_publisher(Twist, "cmd_vel", 10)
    manager.create_subscription(PoseStamped, "position", self.odom_cb, 1)
    self.manager = manager
    self.robot_pose = (None, None)
    self.linear_speed = 0.1 #m/s
    self.target_y = ZoneMeasurements.BERM_OFFSET_Y

    self.pid = ParameterizedPIDController("approach_berm_pid", manager, kp=0.1, ki=0.001, kd=0.0, max_output=self.linear_speed)

    self.berm_up = ZoneMeasurements.BERM_OFFSET_Y < 0 # greater y is toward starting zone
  
  def odom_cb(self, msg:PoseStamped):
    self.robot_pose = (
        msg.pose.position.x,
        msg.pose.position.y,
    )
    
  def start(self):
    if not self.berm_up: # berm y > 0
      self.target_y = ZoneMeasurements.BERM_OFFSET_Y - OFFSET_FROM_CENTER
    else: # berm y <= 0
      self.target_y = ZoneMeasurements.BERM_OFFSET_Y + OFFSET_FROM_CENTER
  
  def periodic(self):
    if self.robot_pose[0] == None:
      return None
    
    if not self.berm_up:
      linear_error = self.robot_pose[1] - self.target_y
    else:
      linear_error = self.target_y - self.robot_pose[1]

    output = Twist()
    if abs(linear_error) < 0.05:
      return Events.SUCCESS
    else:
      output.linear.x = self.pid.calculate(linear_error, 0.1, 0) #self.linear_speed if not self.backwards else -self.linear_speed

    self.cmd_vel_publisher.publish(output)
    return None
  
  def exit(self, event):
    self.cmd_vel_publisher.publish(Twist())

class ApproachBermBackwards(State):    
  def __init__(self, backwards = False):
    self.backwards = backwards

  def setup(self, manager:Node):
    self.cmd_vel_publisher = manager.create_publisher(Twist, "cmd_vel", 10)
    manager.create_subscription(PoseStamped, "position", self.odom_cb, 1)
    self.manager = manager
    self.robot_pose = (None, None)
    self.linear_speed = 0.1 #m/s
    self.target_y = ZoneMeasurements.BERM_OFFSET_Y

    self.pid = ParameterizedPIDController("approach_berm_pid", manager, kp=0.1, ki=0.001, kd=0.0, max_output=self.linear_speed)

    self.berm_up = ZoneMeasurements.BERM_OFFSET_Y < 0 # greater y is toward starting zone
  
  def odom_cb(self, msg:PoseStamped):
    self.robot_pose = (
        msg.pose.position.x,
        msg.pose.position.y,
    )
    
  def start(self):
    if not self.berm_up: # berm y > 0
      self.target_y = ZoneMeasurements.BERM_OFFSET_Y - OFFSET_FROM_CENTER
    else: # berm y <= 0
      self.target_y = ZoneMeasurements.BERM_OFFSET_Y + OFFSET_FROM_CENTER
  
  def periodic(self):
    if self.robot_pose[0] == None:
      return None
    
    if not self.berm_up:
      linear_error = self.robot_pose[1] - self.target_y
    else:
      linear_error = self.target_y - self.robot_pose[1]

    output = Twist()
    if abs(linear_error) < 0.05:
      return Events.SUCCESS
    else:
      output.linear.x = -self.pid.calculate(linear_error, 0.1, 0) #self.linear_speed if not self.backwards else -self.linear_speed

    self.cmd_vel_publisher.publish(output)
    return None
  
  def exit(self, event):
    self.cmd_vel_publisher.publish(Twist())