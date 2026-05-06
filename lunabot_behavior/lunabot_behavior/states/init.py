from rclpy.time import Duration
from std_msgs.msg import Bool
from tf2_ros import Time, TransformBroadcaster, TransformListener, Buffer
from lunabot_behavior.state import Events, State
from apriltag_msgs.msg import AprilTagDetectionArray, AprilTagDetection
from std_srvs.srv import Empty
from enum import Enum, auto
from geometry_msgs.msg import Twist

class Direction(Enum):
  NORTH = auto()
  SOUTH = auto()
  EAST = auto()
  WEST = auto()

direction = None

# sim id - 368
# irl id = 173
INIT_TAG_ID_1 = 173
INIT_TAG_ID_2 = 301

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
    self.manager = manager

    self.tf_buf = Buffer()
    self.tf_listener = TransformListener(self.tf_buf, self.manager)
    self.tf_broadcaster = TransformBroadcaster(self.manager)
    self.has_reset = False

  def tag_cb(self, detections: AprilTagDetectionArray):
    global direction
    self.detections[detections.header.frame_id] = detections
    if "mini/d455_front" in detections.header.frame_id and len(detections.detections) > 0:
      direction = Direction.NORTH if detections.detections[0].id == INIT_TAG_ID_2 else Direction.EAST
    elif "mini/d455_back" in detections.header.frame_id  and any(detection.id == INIT_TAG_ID_1 for detection in detections.detections):
      self.can_see_main_bot = True
    elif "d455_front" in detections.header.frame_id and len(detections.detections) > 0:
      direction = Direction.SOUTH if detections.detections[0].id == INIT_TAG_ID_2 else Direction.WEST

  def periodic(self):
    self.manager.get_logger().info(f"SetupMap: can see main: {self.can_see_main_bot}, dir: {direction}")
    if self.can_see_main_bot and self.is_main and (direction == Direction.NORTH or direction == Direction.EAST):
      mini_detections = self.detections["mini/d455_front_color_optical_frame"]
      mini_detections.header.frame_id = "deposition_apriltag_small_optical_frame"
      try:
        main_to_tag = self.tf_buf.lookup_transform("main_deposition_small", "tag36h11:582" if direction == Direction.EAST else "tag36h11:401", Time())
        main_to_tag.header.frame_id = "deposition_apriltag_small_optical_frame"
        main_to_tag.child_frame_id = "tag36h11:482" if direction == Direction.EAST else "tag36h11:301"
        self.tf_broadcaster.sendTransform(main_to_tag)
        self.detections_pub.publish(mini_detections)
      except Exception as e:
        self.manager.get_logger().warn(f"failed to send detection: {e}")
        self.ready_time = None
    if self.can_see_main_bot and not self.is_main and (direction == Direction.SOUTH or direction == Direction.WEST):
      main_detections = self.detections["d455_front_color_optical_frame"]
      main_detections.header.frame_id = "main_deposition_small"
      main_detections.detections[0].id += 100
      try:
        main_to_tag = self.tf_buf.lookup_transform("deposition_apriltag_small_optical_frame", "tag36h11:482" if direction == Direction.WEST else "tag36h11:301", Time())
        main_to_tag.header.frame_id = "main_deposition_small"
        main_to_tag.child_frame_id = "tag36h11:582" if direction == Direction.WEST else "tag36h11:401"
        self.tf_broadcaster.sendTransform(main_to_tag)
        self.detections_pub.publish(main_detections)
      except Exception as e:
        self.manager.get_logger().warn(f"failed to send detection: {e}")
        self.ready_time = None
    if not self.can_see_main_bot:
      self.ready_time = None
    elif self.ready_time is None:
      self.ready_time = self.manager.get_clock().now()

    if self.ready_time is not None and self.manager.get_clock().now() - self.ready_time > Duration(seconds=2) and not self.has_reset:
      self.manager.get_logger().info("sending map reset")
      self.trigger_new_map_srv.call_async(Empty.Request())
      self.has_reset = True

    if self.ready_time is not None and self.manager.get_clock().now() - self.ready_time > Duration(seconds=10):
      return Events.SUCCESS

class InitRetreat(State):
  def __init__(self, is_main: bool, speed: float, duration: float):
    self.is_main = is_main
    self.speed = speed
    self.duration = duration

  def setup(self, manager):
    self.manager = manager
    self.cmd_vel_publisher = manager.create_publisher(Twist, "cmd_vel", 10)
    self.ready_pub = manager.create_publisher(Bool, "/init/ready", 10)
    self.ready_sub = manager.create_subscription(Bool, "/init/ready", self.ready_cb, 10)
    self.ready = False
    self.stalled = False
    self.elapsed = 0

  def ready_cb(self, ready: Bool):
    self.ready = ready.data

  def start(self):
    self.is_moving = (self.is_main and (direction == Direction.NORTH or direction == Direction.EAST)) or\
      (not self.is_main and (direction == Direction.SOUTH or direction == Direction.WEST))
    self.starting_time = self.manager.get_clock().now()
    if self.stalled:
      self.duration -= self.elapsed
      self.stalled = False

  def periodic(self):
    if self.is_moving:
      output = Twist()
      output.linear.x = self.speed
      self.cmd_vel_publisher.publish(output)
      if self.manager.get_clock().now() - self.starting_time > Duration(seconds=self.duration):
        self.ready_pub.publish(Bool(data = True))

        if (direction == Direction.EAST and self.is_main):
          return Events.SUCCESS_AND_DONT_MINE

        return Events.SUCCESS
    elif self.ready:
      if (direction == Direction.EAST and self.is_main):
        return Events.SUCCESS_AND_DONT_MINE
      
      return Events.SUCCESS

  def exit(self, event):
    self.duration += self.elapsed
    if event is Events.STALL:
      self.stalled = True
      self.elapsed += self.manager.get_clock().now().seconds_nanoseconds()[0] - self.starting_time.seconds_nanoseconds()[0]
    else:
      self.elapsed = 0
    self.cmd_vel_publisher.publish(Twist())
