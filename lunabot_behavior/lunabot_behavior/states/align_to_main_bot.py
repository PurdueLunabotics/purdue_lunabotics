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
from tf2_ros import Buffer, TransformListener, TransformStamped
from tf2_geometry_msgs import do_transform_pose
from typing import Literal
import numpy as np
import math

# change for sim
# sim apriltag - 368
# irl tag - 126
DEPOSITION_APRILTAG_ID = 126

class AlignToMainBotState(State):
  def __init__(self):

    # (x, y, theta)
    self.robot_pose = None

    self.tf_buffer = Buffer()
    self.tf_listener = None

    self.apriltag_detections: AprilTagDetectionArray = None

    self.node: Node = None

    # PID for angular alignment
    self.P = 1.5
    self.I = 0.05
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

    self.SEARCH_SPEED = 0.3 # rad/s

    # in rad, how aligned before it returns success
    self.ANGULAR_ALIGN_THRESHOLD = 0.02
    # how many times we have been well aligned
    self.success_count = 0
    # how many times in a row before we're sure
    self.SUCCESS_THRESHOLD = 30

  def setup(self, manager: Node):
    self.cmd_vel_publisher = manager.create_publisher(Twist, "/mini/cmd_vel", 10)
    self.visual_publisher = manager.create_publisher(Marker, "/mini/align_goal", 10)
    self.apriltag_offset_publisher = manager.create_publisher(TransformStamped, "/behavior/mini_apriltag_offset", 10)
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
    self.node.get_logger().info("Behavior: Align to main bot: starting search" )
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
        self.resetPID()
        self.success_count = 0
        self.node.get_logger().info("Behavior: Align to main bot: starting align")
        return None

      self.publish_angular_velocity(self.SEARCH_SPEED)

    elif (self.internal_state == 'align'):
      # Align to face where the apriltag is

      if (self.isApriltagPresent()):

        try:
          # get where the apriltag is
          apriltag_to_minimap_transform = self.tf_buffer.lookup_transform("mini/map", "main_deposition", rclpy.time.Time(seconds=0), Duration(nanoseconds=500_000))

          # get base-link in apriltag frame
          baselink_to_apriltag_transform = self.tf_buffer.lookup_transform("deposition_apriltag_optical_frame","base_link", rclpy.time.Time(seconds=0), Duration(nanoseconds=500_000))

          baselink_in_apriltag_frame = Pose()
          baselink_in_apriltag_frame.position.x = baselink_to_apriltag_transform.transform.translation.x
          baselink_in_apriltag_frame.position.y = baselink_to_apriltag_transform.transform.translation.y
          baselink_in_apriltag_frame.position.z = baselink_to_apriltag_transform.transform.translation.z

          # translate base link to mini/map frame
          baselink_in_minimap_frame = do_transform_pose(baselink_in_apriltag_frame, apriltag_to_minimap_transform)

          # get the transform of the minibot in the apriltag frame (this will be passed to the next state, and to main bot)
          mini_in_apriltag_frame_transform = self.tf_buffer.lookup_transform("main_deposition", "mini/base_link",  rclpy.time.Time(seconds=0), Duration(nanoseconds=500_000))

          xDiff = self.robot_pose[0] - baselink_in_minimap_frame.position.x
          yDiff = self.robot_pose[1] - baselink_in_minimap_frame.position.y

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
              self.publish_offset_transform(mini_in_apriltag_frame_transform)
              return Events.SUCCESS
            
          else:
            self.success_count = 0

          velocity = self.runPID(error)
          self.publish_angular_velocity(velocity)

          self.visualize_alignment(baselink_in_minimap_frame)

        except Exception as e:
          pass
          # self.node.get_logger().info(f"{e}")
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


  def visualize_alignment(self, main_bot_pose: Pose):
    marker = Marker()
    marker.header.frame_id = "mini/map"
    marker.header.stamp = self.node.get_clock().now().to_msg()
    marker.id = 3681
    marker.ns = "align"
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD

    start = Point(x=self.robot_pose[0], y=self.robot_pose[1], z=0.3)
    end = Point(x=main_bot_pose.position.x, y=main_bot_pose.position.y, z=main_bot_pose.position.z)

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

  def publish_offset_transform(self, transform: TransformStamped):
    for i in range(5):
      self.apriltag_offset_publisher.publish(transform)
  
  def exit(self, event):
    # stop moving
    self.cmd_vel_publisher.publish(Twist())

    
