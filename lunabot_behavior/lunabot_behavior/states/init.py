from rclpy.time import Duration
from tf2_ros import Time, TransformBroadcaster, TransformListener, Buffer
from lunabot_behavior.state import Events, State
from apriltag_msgs.msg import AprilTagDetectionArray, AprilTagDetection
from std_srvs.srv import Empty
from enum import Enum, auto

class Direction(Enum):
  NORTH = auto()
  SOUTH = auto()
  EAST = auto()
  WEST = auto()

class SetupMap(State):
  def __init__(self, is_main: bool):
    self.is_main = is_main
    self.detections: dict[str, AprilTagDetectionArray] = {}

  def setup(self, manager):
    self.main_front_detections_sub = manager.create_subscription(AprilTagDetectionArray, "/d455_front/detections", self.tag_cb, 10)
    self.main_back_detections_sub = manager.create_subscription(AprilTagDetectionArray, "/d455_back/detections", self.tag_cb, 10)
    self.mini_front_detections_sub = manager.create_subscription(AprilTagDetectionArray, "/mini/d455_front/detections", self.tag_cb, 10)
    self.mini_back_detections_sub = manager.create_subscription(AprilTagDetectionArray, "/mini/d455_back/detections", self.tag_cb, 10)

    self.detections_pub = manager.create_publisher(AprilTagDetectionArray, "rtabmap/apriltag/detections", 1)
    self.trigger_new_map_srv = manager.create_client(Empty, "rtabmap/rtabmap/trigger_new_map")
    self.trigger_new_map_srv.wait_for_service()

    self.can_see_main_bot = False
    self.ready_time = None
    self.direction = None
    self.manager = manager

    self.tf_buf = Buffer()
    self.tf_listener = TransformListener(self.tf_buf, self.manager)
    self.tf_broadcaster = TransformBroadcaster(self.manager)

  def tag_cb(self, detections: AprilTagDetectionArray):
    self.detections[detections.header.frame_id] = detections
    if detections.header.frame_id == "d455_front_rgb_link" and len(detections.detections) > 0:
      self.direction = Direction.SOUTH if detections.detections[0].id == 11 else Direction.WEST
    elif detections.header.frame_id == "mini/d455_front_rgb_link" and len(detections.detections) > 0:
      self.direction = Direction.NORTH if detections.detections[0].id == 11 else Direction.EAST
    elif detections.header.frame_id == "mini/d455_back_rgb_link" and any(detection.id == 368 for detection in detections.detections):
      self.can_see_main_bot = True

  def periodic(self):
    print(f"dir {self.direction}, can see main bot {self.can_see_main_bot}")
    if self.can_see_main_bot and self.is_main and (self.direction == Direction.NORTH or self.direction == Direction.EAST):
      mini_detections = self.detections["mini/d455_front_rgb_link"]
      mini_detections.header.frame_id = "deposition_apriltag_optical_frame"
      try:
        main_to_tag = self.tf_buf.lookup_transform("main_deposition", "tag36h11:107" if self.direction == Direction.EAST else "tag36h11:111", Time())
        main_to_tag.header.frame_id = "deposition_apriltag_optical_frame"
        main_to_tag.child_frame_id = "tag36h11:7" if self.direction == Direction.EAST else "tag36h11:11"
        self.tf_broadcaster.sendTransform(main_to_tag)
        self.detections_pub.publish(mini_detections)
      except Exception as e:
        self.manager.get_logger().warn(f"failed to send detection: {e}")
        self.ready_time = None
    if self.can_see_main_bot and not self.is_main and (self.direction == Direction.SOUTH or self.direction == Direction.WEST):
      main_detections = self.detections["d455_front_rgb_link"]
      main_detections.header.frame_id = "main_deposition"
      main_detections.detections[0].id += 100
      try:
        main_to_tag = self.tf_buf.lookup_transform("deposition_apriltag_optical_frame", "tag36h11:7" if self.direction == Direction.WEST else "tag36h11:11", Time())
        main_to_tag.header.frame_id = "main_deposition"
        main_to_tag.child_frame_id = "tag36h11:107" if self.direction == Direction.WEST else "tag36h11:111"
        self.tf_broadcaster.sendTransform(main_to_tag)
        self.detections_pub.publish(main_detections)
      except Exception as e:
        self.manager.get_logger().warn(f"failed to send detection: {e}")
        self.ready_time = None
    if not self.can_see_main_bot:
      self.ready_time = None
    elif self.ready_time is None:
      self.ready_time = self.manager.get_clock().now()

    if self.ready_time is not None and self.manager.get_clock().now() - self.ready_time > Duration(seconds=10):
      self.trigger_new_map_srv.call_async(Empty.Request())
      print("success")
      return Events.SUCCESS
