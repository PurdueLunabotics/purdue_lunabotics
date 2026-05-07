from lunabot_behavior.state import State, Events
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from tf_transformations import euler_from_quaternion
from lunabot_control.pid_controller import ParameterizedPIDController
import numpy as np


class AlignToAngle(State):
  def __init__(self, angle, **kwargs):
    self.target_angle = np.deg2rad(angle) % (2 * np.pi)
    
  def setup(self, manager:Node):
    self.cmd_vel_publisher = manager.create_publisher(Twist, "cmd_vel", 10)
    manager.create_subscription(PoseStamped, "position", self.odom_cb, 1)
    self.manager = manager
    self.robot_pose: None | tuple[float, float, float] = None
    self.angular_speed = np.deg2rad(30) #degrees/sec -> rad/sec
    self.tolerance = np.deg2rad(3)
    self.pid = ParameterizedPIDController("angle", manager, kp=5.0, ki=0.0, kd=0.0, max_output=0.523)
  
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
        angles[2] % (2 * np.pi),  # 0 to 2pi
    )
    
  def start(self):
    pass
  
  def periodic(self):
    if self.robot_pose is None:
      return None
    angular_error = self.robot_pose[2] - self.target_angle
    if angular_error > np.pi:
      angular_error -= 2 * np.pi
    elif angular_error < -np.pi:
      angular_error += 2 * np.pi
    
    if np.abs(angular_error) < self.tolerance:
      return Events.SUCCESS

    output = Twist()
    output.angular.z = self.pid.calculate(angular_error, 0.1, 0)
    self.cmd_vel_publisher.publish(output)
    return None
  
  def exit(self, event):
    self.cmd_vel_publisher.publish(Twist())
    
