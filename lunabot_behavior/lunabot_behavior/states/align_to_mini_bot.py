from lunabot_behavior.state import Events, State
from rclpy.node import Node
import rclpy
import rclpy.time
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import Twist, PoseStamped, Point, Pose
from visualization_msgs.msg import Marker
from apriltag_msgs.msg import AprilTagDetectionArray
from tf_transformations import euler_from_quaternion
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener, TransformStamped
from typing import Literal
import numpy as np
import math

class AlignToMiniBotState(State):
  def __init__(self):

    # (x, y, theta)
    self.robot_pose = None

    self.tf_buffer = Buffer()
    self.tf_listener = None

    # transform of the mini bot in the apriltag frame
    self.mini_transform_offset: TransformStamped = None

    # pose of minibot in the map frame
    self.mini_pose: Pose = None

    self.node: Node = None

    # PID for angular alignment
    self.P = 3
    self.I = 0
    self.D = 1

    self.last_error = None
    self.total_error = 0
    self.last_time = None

    # in rad, how aligned before it returns success
    self.ANGULAR_ALIGN_THRESHOLD = 0.1
    # how many times we have been well aligned
    self.success_count = 0
    # how many times in a row before we're sure
    self.SUCCESS_THRESHOLD = 30

  def setup(self, manager: Node):
    self.cmd_vel_publisher = manager.create_publisher(Twist, "/cmd_vel", 10)
    self.visual_publisher = manager.create_publisher(Marker, "/align_goal", 10)
    manager.create_subscription(PoseStamped, "/position", self.odom_callback, 10)
    manager.create_subscription(TransformStamped, "/behavior/mini_apriltag_offset", self.transform_callback, 10)
    self.node = manager

    self.tf_listener = TransformListener(self.tf_buffer, manager)
  
  def odom_callback(self, msg: PoseStamped):
    angles = euler_from_quaternion([
                        msg.pose.orientation.x,
                        msg.pose.orientation.y,
                        msg.pose.orientation.z,
                        msg.pose.orientation.w,
                    ])
    
    self.robot_pose = (
        msg.pose.position.x,
        msg.pose.position.y,
        angles[2],
    )

  def transform_callback(self, msg: TransformStamped):
    self.mini_transform_offset = msg

  def start(self):
    self.node.get_logger().info("Behavior: Align to mini bot: starting alignment" )
    self.resetPID()
    self.success_count = 0
    self.mini_pose = None
    self.mini_transform_offset = None
  
  def periodic(self):

    # if we don't have the target pose yet, find it
    if (self.mini_pose is None):

      # if we don't have this transform, we can't do anything
      if (self.mini_transform_offset is None):
        return None

      try:
        transform = self.tf_buffer.lookup_transform("map", "deposition_apriltag_optical_frame", rclpy.time.Time(seconds=0), Duration(nanoseconds=500_000))

        mini_in_apriltag_frame = Pose()
        mini_in_apriltag_frame.position.x = self.mini_transform_offset.transform.translation.x
        mini_in_apriltag_frame.position.y = self.mini_transform_offset.transform.translation.y
        mini_in_apriltag_frame.position.z = self.mini_transform_offset.transform.translation.z
        mini_in_apriltag_frame.orientation.x = self.mini_transform_offset.transform.rotation.x
        mini_in_apriltag_frame.orientation.y = self.mini_transform_offset.transform.rotation.y
        mini_in_apriltag_frame.orientation.z = self.mini_transform_offset.transform.rotation.z
        mini_in_apriltag_frame.orientation.w = self.mini_transform_offset.transform.rotation.w

        self.mini_pose = do_transform_pose(mini_in_apriltag_frame, transform)
      except Exception as e:
        # pass
        print("waiting for transform", e)

    # if we do have the target pose, align to it
    else:

      xDiff = self.robot_pose[0] - self.mini_pose.position.x
      yDiff = self.robot_pose[1] - self.mini_pose.position.y

      goal_angle = math.atan2(yDiff, xDiff)
      current_angle = self.robot_pose[2]

      error = goal_angle - current_angle
      error = (error + np.pi) % (2 * np.pi) - np.pi

      # if aligned, increment a counter. At the threshold, alignment is done
      if (abs(error) < self.ANGULAR_ALIGN_THRESHOLD):
        self.success_count +=1

        # if aligned, return success for next state, and the offset from mini to the apriltag (used by main bot)
        if (self.success_count >= self.SUCCESS_THRESHOLD):
          self.remove_marker()
          return Events.SUCCESS
          
      else:
        self.success_count = 0

      velocity = self.runPID(error)
      self.publish_angular_velocity(velocity)

      self.visualize_alignment(self.mini_pose)

  
  def runPID(self, error: float):

    if (self.last_time is not None):
      dt = self.node.get_clock().now() - self.last_time
    else:
      dt = Duration(seconds=0)
    
    # time in seconds
    dt = dt.nanoseconds / 1000000000
    
    if (self.last_error is not None):
      change = (error - self.last_error) / dt
    else:
      change = 0

    self.last_error = error
    self.last_time = self.node.get_clock().now()

    self.total_error += error * dt

    return self.P * error + change * self.D + self.total_error * self.I

  def resetPID(self):
    self.total_error = 0
    self.last_error = None
    self.last_time = None

  def publish_angular_velocity(self, velocity: float):
    vel = Twist()
    vel.angular.z = velocity

    self.cmd_vel_publisher.publish(vel)

  def visualize_alignment(self, mini_pose: Pose):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = self.node.get_clock().now().to_msg()
    marker.id = 3683
    marker.ns = "align"
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD

    start = Point(x=self.robot_pose[0], y=self.robot_pose[1], z=0.3)
    end = Point(x=mini_pose.position.x, y=mini_pose.position.y, z=0.3)

    marker.points.append(start)
    marker.points.append(end)

    marker.color.r = 0.95
    marker.color.g = 0.8
    marker.color.b = 0.0
    marker.color.a = 0.9
    marker.scale.x = 0.035
    self.visual_publisher.publish(marker)
  
  def remove_marker(self):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = self.node.get_clock().now().to_msg()
    marker.id = 3683
    marker.ns = "align"
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.DELETE
    self.visual_publisher.publish(marker)
  
  def exit(self):
    # stop moving
    self.cmd_vel_publisher.publish(Twist())

    # delete transform so it's not reused
    self.mini_transform_offset = None
    