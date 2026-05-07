from lunabot_behavior.state import Events, State
from rclpy.node import Node
import rclpy
import rclpy.time
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import Twist, PoseStamped, Point, Pose
from std_msgs.msg import Bool
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
MINI_DEP_APRILTAG_ID = 173

class ApproachMiniState(State):
  def __init__(self):

    self.tf_buffer = Buffer()
    self.tf_listener = None

    self.apriltag_detections: AprilTagDetectionArray = None

    self.node: Node = None

    # PID for linear alignment
    self.P = 1
    self.I = 0.001
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

    self.MAX_SPEED = 0.25

    self.TIMEOUT_TIME = 30 # in seconds, how long to wait before giving up and continuing

  def setup(self, manager: Node):
    self.cmd_vel_publisher = manager.create_publisher(Twist, "/cmd_vel", 10)
    self.aligned_msg_publisher = manager.create_publisher(Bool, "/behavior/main_approached", 10)
    manager.create_subscription(AprilTagDetectionArray, "/mini/d455_back/detections", self.apriltag_callback, 10)
    self.node = manager

    self.tf_listener = TransformListener(self.tf_buffer, manager)
  

  def apriltag_callback(self, msg: AprilTagDetectionArray):
    self.apriltag_detections = msg
    
  def start(self):
    self.node.get_logger().info("Behavior: Approaching mini bot" )
    self.apriltag_detections = None
    self.resetPID()
    self.lost_count = 0
    self.success_count = 0

    self.start_time = self.node.get_clock().now()
  
  def periodic(self):

    elapsed_time = self.node.get_clock().now() - self.start_time
    elapsed_time = elapsed_time.nanoseconds / 1_000_000_000  # convert to seconds

    if (elapsed_time > self.TIMEOUT_TIME):
      # if we timeout, return success (assume we're done)
      self.publish_aligned_msg()
      return Events.SUCCESS

    apriltag_present, detections = self.isApriltagPresent()
    if (apriltag_present):
        # print(self.apriltag_detections)

        try:
            if (DEPOSITION_APRILTAG_ID in detections): # prioritize big tag
              apriltag_in_camera_frame = self.tf_buffer.lookup_transform("mini/d455_back_rgb_link", "main_deposition", rclpy.time.Time(seconds=0), Duration(nanoseconds=500_000))
            else:
              apriltag_in_camera_frame = self.tf_buffer.lookup_transform("mini/d455_back_rgb_link", "main_deposition_small", rclpy.time.Time(seconds=0), Duration(nanoseconds=500_000))

            distance = apriltag_in_camera_frame.transform.translation.z 

            error = self.DISTANCE_GOAL - distance

            # if aligned, increment a counter. at the threshold, alignment is done
            if (abs(error) < self.GOAL_THRESHOLD):
                self.success_count += 1

            # if aligned, return success for next state, and the transition message for the next state
            if (self.success_count >= self.SUCCESS_THRESHOLD):
                self.publish_aligned_msg()
                return Events.SUCCESS
            
            velocity = self.runPID(error)
            velocity = np.clip(velocity, -self.MAX_SPEED, self.MAX_SPEED)
            self.publish_linear_velocity(velocity)

        except Exception as e:
          pass
    
    else:
      # if we can't see apriltag enough times in a row, exit this node. for now, assume success
      self.lost_count+=1

      if (self.lost_count >= self.LOST_APRILTAG_THRESHOLD):
        self.lost_count = 0
        self.node.get_logger().warn("Behavior: Exiting approach due to LOST apriltag!")
        self.publish_aligned_msg()
        return Events.SUCCESS

    return None
  
  def isApriltagPresent(self):
    apriltag_present = False
    detections = []
    if (self.apriltag_detections != None):
      for detection in self.apriltag_detections.detections:
        if (detection.id == DEPOSITION_APRILTAG_ID or detection.id == MINI_DEP_APRILTAG_ID):
          apriltag_present = True
          if (detection.id not in detections):
            detections.append(detection.id)

    return apriltag_present, detections
  
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

  def publish_aligned_msg(self):
    msg = Bool()
    msg.data=True

    for i in range(10):
      self.aligned_msg_publisher.publish(msg)

  
  def exit(self, event):
    # stop moving
    self.cmd_vel_publisher.publish(Twist())

    
