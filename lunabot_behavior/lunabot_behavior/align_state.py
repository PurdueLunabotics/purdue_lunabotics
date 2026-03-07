from state import State, Events
from rclpy.node import Node
import rclpy
import rclpy.time
from rclpy.duration import Duration
from geometry_msgs.msg import Twist, PoseStamped
from visualization_msgs.msg import Marker
from apriltag_msgs.msg import AprilTagDetectionArray
from tf_transformations import euler_from_quaternion
from tf2_ros import Buffer, TransformListener
import numpy as np
import math

DEPOSITION_APRILTAG_ID = 368

class AlignToMainBotState(State):
  def __init__(self):
    self.robot_pose = None

    self.tf_buffer = Buffer()
    self.tf_listener = None

    self.apriltag_detections: AprilTagDetectionArray = None

    self.node = None

  def setup(self, manager: Node):
    self.cmd_vel_publisher = manager.create_publisher(Twist, "cmd_vel", 10)
    self.visual_publisher = manager.create_publisher(Marker, "alignGoal", 10)
    manager.create_subscription(PoseStamped, "/mini/position", self.odom_callback, 10)
    self.node = manager

    manager.create_subscription(AprilTagDetectionArray, "/mini/d455_back/detections", self.apriltag_callback, 10)

    self.tf_listener = TransformListener(self.tf_buffer, manager)
  
  def odom_callback(self, msg: PoseStamped):
    print("here...")
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
    self.start_time = rclpy
    print("my cool state started! YAY")
  
  def periodic(self):
    # print(self.robot_pose)

    apriltag_present = False
    if (self.apriltag_detections != None):
      for detection in self.apriltag_detections.detections:
        if (detection.id == DEPOSITION_APRILTAG_ID):
          apriltag_present = True

    if (apriltag_present):
      try:
        transform = self.tf_buffer.lookup_transform("mini/map", "main_deposition", rclpy.time.Time(seconds=0), Duration(nanoseconds=500_000))

        xDiff = self.robot_pose[0] - transform.transform.translation.x
        yDiff = self.robot_pose[1] - transform.transform.translation.y

        print(math.atan2(yDiff, xDiff), "vs", self.robot_pose[2])

        marker = Marker()
        # Set the frame
        marker.header.frame_id = "mini/map"
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.id = 0
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

      except Exception as e:
        print("waiting on transform...")

    return None
  
  def exit(self):
    # self.cmd_vel_publisher.publish(Twist())
    print("Exited, YAY!")
    