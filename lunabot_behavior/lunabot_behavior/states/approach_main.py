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

DEPOSITION_APRILTAG_ID = 368

class ApproachMainState(State):
  def __init__(self):

    # (x, y, theta)
    self.robot_pose = None

    self.tf_buffer = Buffer()
    self.tf_listener = None

    self.apriltag_detections: AprilTagDetectionArray = None

    self.node: Node = None

    # PID for linear alignment
    self.P = 1
    self.I = 0
    self.D = 0

    self.last_error = None
    self.total_error = 0
    self.last_time = None

    # how many times we've lost apriltag.
    self.lost_count = 0
    # how many times we tolerate before giving up
    self.LOST_APRILTAG_THRESHOLD = 15

    # in meters, what distance between robots we are looking for
    self.DISTANCE_GOAL = 0.12
    # in meters, how close to this goal we need to get before returning success
    self.GOAL_THRESHOLD = 0.01
    # how many times we have been aligned
    self.success_count = 0
    # how many times in a row before we're sure
    self.SUCCESS_THRESHOLD = 30

  def setup(self, manager: Node):
    self.cmd_vel_publisher = manager.create_publisher(Twist, "/mini/cmd_vel", 10)
    self.visual_publisher = manager.create_publisher(Marker, "/mini/approach_visual", 10)
    manager.create_subscription(PoseStamped, "/mini/position", self.odom_callback, 10)
    manager.create_subscription(AprilTagDetectionArray, "/mini/d455_back/detections", self.apriltag_callback, 10)
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

  def apriltag_callback(self, msg: AprilTagDetectionArray):
    self.apriltag_detections = msg
    
  def start(self):
    self.node.get_logger().info("Behavior: Approaching main bot" )
    self.apriltag_detections = None
    self.resetPID()
    self.lost_count = 0
    self.success_count = 0
  
  def periodic(self):

    if (self.isApriltagPresent()):
        # print(self.apriltag_detections)

        try:
            apriltag_in_camera_frame = self.tf_buffer.lookup_transform("mini/d455_back_rgb_link", "main_deposition", rclpy.time.Time(seconds=0), Duration(nanoseconds=500_000))

            distance = apriltag_in_camera_frame.transform.translation.z 
            print(distance)

            error = self.DISTANCE_GOAL - distance

            # if aligned, increment a counter. at the threshold, alignment is done
            if (abs(error) < self.GOAL_THRESHOLD):
                self.success_count += 1

            # if aligned, return success for next state, and the offset from mini to the apriltag (used by main bot)
            if (self.success_count >= self.SUCCESS_THRESHOLD):
                self.remove_marker()
                return Events.SUCCESS
            
            velocity = self.runPID(error)
            self.publish_linear_velocity(velocity)

        except Exception as e:
          pass
    
    else:
      # if we can't see apriltag enough times in a row, exit this node. for now, assume success
      self.lost_count+=1

      if (self.lost_count >= self.LOST_APRILTAG_THRESHOLD):
        self.lost_count = 0
        self.node.get_logger().info("Behavior: Exiting approach due to LOST apriltag!")
        return Events.SUCCESS

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

  def publish_linear_velocity(self, velocity: float):
    vel = Twist()
    vel.linear.x = velocity

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

  
  def exit(self):
    # stop moving
    self.cmd_vel_publisher.publish(Twist())

    