from lunabot_behavior.state import Events, State
from rclpy.node import Node
import rclpy
import rclpy.time
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import Twist, PoseStamped, Point
from visualization_msgs.msg import Marker
from apriltag_msgs.msg import AprilTagDetectionArray
from tf_transformations import euler_from_quaternion
from tf2_ros import Buffer, TransformListener, TransformStamped
from typing import Literal
import numpy as np
import math

DEPOSITION_APRILTAG_ID = 368

class AlignToMainBotState(State):
  def __init__(self):
    # (x, y, theta)
    self.robot_pose = None

    self.tf_buffer = Buffer()
    self.tf_listener = None

    self.apriltag_detections: AprilTagDetectionArray = None

    self.node: Node = None

    # PID for angular alignment
    self.P = 1
    self.I = 0
    self.D = 0

    self.last_error = None
    self.total_error = 0
    self.last_time = None

    # Search = look for apriltag, Align = align to target
    self.internal_state: Literal['search', 'align'] = 'search'

    # how many times we've lost apriltag.
    self.lost_count = 0
    # how many times we tolerate before going to 'search' mode
    self.LOST_APRILTAG_THRESHOLD = 15

    self.SEARCH_SPEED = 0.6 # rad/s

    # in rad, how aligned before it retunrs success
    self.ANGULAR_ALIGN_THRESHOLD = 0.1
    # how many times we have been well aligned
    self.success_count = 0
    # how many times in a row before we're sure
    self.SUCCESS_THRESHOLD = 30

  def setup(self, manager: Node):
    self.cmd_vel_publisher = manager.create_publisher(Twist, "/mini/cmd_vel", 10)
    self.visual_publisher = manager.create_publisher(Marker, "/mini/align_goal", 10)
    manager.create_subscription(PoseStamped, "/mini/position", self.odom_callback, 10)
    self.node = manager

    manager.create_subscription(AprilTagDetectionArray, "/mini/d455_back/detections", self.apriltag_callback, 10)

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

  def apriltag_callback(self, msg: AprilTagDetectionArray):
    self.apriltag_detections = msg
    
  def start(self):
    self.node.get_logger().info("Behavior: Align to main bot: starting search", )
    self.internal_state = 'search'
    self.apriltag_detections = None
    self.resetPID()
    self.lost_count = 0
    self.success_count = 0
  
  def periodic(self):

    if (self.internal_state == 'search'):
      # spin in circle until you see main bot's apriltag

      if (self.isApriltagPresent()):
        self.internal_state = 'align'
        self.success_count = 0
        self.node.get_logger().info("Behavior: Align to main bot: starting align")
        return None

      self.publish_angular_velocity(self.SEARCH_SPEED)

    elif (self.internal_state == 'align'):
      # Align to face where the apriltag is

      if (self.isApriltagPresent()):

        try:
          # get where the apriltag is
          transform = self.tf_buffer.lookup_transform("mini/map", "main_deposition", rclpy.time.Time(seconds=0), Duration(nanoseconds=500_000))

          xDiff = self.robot_pose[0] - transform.transform.translation.x
          yDiff = self.robot_pose[1] - transform.transform.translation.y

          goal_angle = math.atan2(yDiff, xDiff)
          current_angle = self.robot_pose[2]

          error = goal_angle - current_angle
          error = (error + np.pi) % (2 * np.pi) - np.pi

          # if aligned, increment a counter. At the threshold, alignment is done
          if (abs(error) < self.ANGULAR_ALIGN_THRESHOLD):
            self.success_count +=1
            if (self.success_count >= self.SUCCESS_THRESHOLD):
              self.remove_marker()
              return Events.SUCCESS
          else:
            self.success_count = 0

          velocity = self.runPID(error)
          self.publish_angular_velocity(velocity)

          self.visualize_alignment(transform)

        except Exception as e:
          pass
          # print("waiting on transform...", e)
      else:
        # if we can't see apriltag in 'align' mode enough times in a row, then we lost it, go back to searching
        self.lost_count+=1
        
        if (self.lost_count >= self.LOST_APRILTAG_THRESHOLD):
          self.node.get_logger().info("Behavior: Align to main bot: lost tag, searching")
          self.internal_state = 'search'
          self.lost_count = 0
          return None

    return None
  
  def isApriltagPresent(self):
    apriltag_present = False
    if (self.apriltag_detections != None):
      for detection in self.apriltag_detections.detections:
        if (detection.id == DEPOSITION_APRILTAG_ID):
          apriltag_present = True

    return apriltag_present
  
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

  def visualize_transform(self, transform: TransformStamped):
    marker = Marker()
    # Set the frame
    marker.header.frame_id = "mini/map"
    marker.header.stamp = self.node.get_clock().now().to_msg()
    marker.id = 368
    marker.type = Marker.ARROW
    marker.action = Marker.ADD

    # Set the position of the point
    marker.pose.position.x = transform.transform.translation.x
    marker.pose.position.y = transform.transform.translation.y
    marker.pose.position.z = transform.transform.translation.z
    marker.pose.orientation.x = transform.transform.rotation.x
    marker.pose.orientation.y = transform.transform.rotation.y
    marker.pose.orientation.z = transform.transform.rotation.z
    marker.pose.orientation.w = transform.transform.rotation.w

    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0  
    marker.scale.x = 0.3
    marker.scale.y = 0.02
    marker.scale.z = 0.02

    self.visual_publisher.publish(marker)

  def visualize_alignment(self, transform: TransformStamped):
    marker = Marker()
    marker.header.frame_id = "mini/map"
    marker.header.stamp = self.node.get_clock().now().to_msg()
    marker.id = 3681
    marker.ns = "align"
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD

    start = Point(x=self.robot_pose[0], y=self.robot_pose[1], z=0.3)
    end = Point(x=transform.transform.translation.x, y=transform.transform.translation.y, z=transform.transform.translation.z)

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
    marker.header.frame_id = "mini/map"
    marker.header.stamp = self.node.get_clock().now().to_msg()
    marker.id = 3681
    marker.ns = "align"
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.DELETE
    self.visual_publisher.publish(marker)
  
  def exit(self):
    # stop moving
    self.cmd_vel_publisher.publish(Twist())

    # delete marker
    marker = Marker()
    marker.action = Marker.DELETE
    marker.id = 368
    self.visual_publisher.publish(marker)
    