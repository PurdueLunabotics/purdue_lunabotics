from state import State, Events
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from tf_transformations import euler_from_quaternion
import numpy as np


class AlignToAngleState(State):
  def __init__(self, angle, **kwargs):
    super().__init__()
    self.target_angle = np.deg2rad(angle)
    
  def setup(self, manager:Node):
    self.cmd_vel_publisher = manager.create_publisher(Twist, "cmd_vel", 10)
    manager.create_subscription(PoseStamped, "odom", self.odom_cb, 1)
    self.manager = manager
    self.robot_pose = (None, None, None)
    self.angular_speed = np.deg2rad(60) #degrees/sec -> rad/sec
  
  def odom_cb(self, msg:PoseStamped):
    angles = euler_from_quaternion(
            [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ]
        )
    self.robot_pose = (
        msg.pose.position.x,
        msg.pose.position.y,
        angles[2],  # -pi to pi
    )
    
  def start(self):
    pass
  
  def periodic(self):
    if self.robot_pose[0] == None:
      return None
    angular_error = self.target_angle - self.robot_pose[2]
    angular_error = (angular_error + np.pi) % (2 * np.pi) - np.pi
    
    if self.robot_pose[2] != None and np.abs(angular_error) < self.tolerance:
      return Events.SUCCESS
    output = Twist()
    if angular_error < 0:
      output.angular.z = self.angular_speed
    else:
      output.angular.z = -self.angular_speed
    self.cmd_vel_publisher.publish(output)
    return None
  
  def exit(self):
    self.cmd_vel_publisher.publish(Twist())
    