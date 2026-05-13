from rclpy.node import Node
from rclpy.task import Future
from rclpy.parameter import Parameter

from nav2_msgs.msg import Costmap
from nav2_msgs.srv import GetCostmap
from geometry_msgs.msg import Point, Twist
from visualization_msgs.msg import Marker
from std_msgs.msg import Header

from lunabot_behavior.states.align_to_angle import AlignToAngle
import lunabot_behavior.zones as zones
from lunabot_behavior.util import CostmapUtil
from lunabot_behavior.state import Events

import shapely.geometry as shp
import numpy as np

LOOKAHEAD_DIST = 1.0 # m - how far the robot will go in its "trenching" (to the tip of excavation)
LETHAL_COST = 220

class AlignTrench(AlignToAngle):
  # =========================
  # main state functions
  # =========================

  def __init__(self, **kwargs):
    self.base_angle = self.get_base_exc_angle()
    super().__init__(self.base_angle)

  def setup(self, manager: Node):
    super().setup(manager)
    self.min_angle = self.base_angle + np.deg2rad(-90)
    self.max_angle = self.base_angle + np.deg2rad(90) + 0.01
    self.angle_step = np.deg2rad(30)
    self.exc_approach_dist = 0.0  # keeps track of how far robot goes on angle before excavating
    self.distance_step = 0.5 # m - how far each excavation step proceeds
    self.frame_id = "map"

    self.manager = manager
    self.costmap_client = manager.create_client(GetCostmap, "global_costmap/get_costmap")
    self.marker_pub = manager.create_publisher(Marker, "/trench_marker", 10)
    self.marker_pub.publish(Marker(action=Marker.DELETEALL, header=Header(frame_id=self.frame_id)))

    self.manager.declare_parameter("exc_approach_dist", self.exc_approach_dist)
    self.manager.declare_parameter("exc_retreat_dist", self.exc_approach_dist)

    self.valid_angles = np.arange(self.min_angle, self.max_angle, self.angle_step).tolist()
    self.manager.get_logger().info(f"Angles: {[np.rad2deg(a) for a in self.valid_angles]}")
    self.valid_angles_updated = False

    self.completed_angles = []  # gets reset on each increment of excavation approach dist

    self.manager.get_logger().info(f"Base angle: {self.base_angle}")

    self.called_costmap = False

    self.no_paths = False

    self.i = 0

  def start(self):
    # set the correct retreat distance
    self.trenching_dist = self.manager.get_parameter("trenching_dist").get_parameter_value().double_value  # value set in trench.py for better readability
    new_retreat_dist = Parameter("exc_retreat_dist", Parameter.Type.DOUBLE, self.exc_approach_dist + self.trenching_dist)
    self.manager.set_parameters([new_retreat_dist])

    # check valid angles again, start with full set of angles in case mistaken obstacles were removed or robot is in a diff position
    self.valid_angles = np.arange(self.min_angle, self.max_angle, self.angle_step).tolist()
    self.marker_pub.publish(Marker(action=Marker.DELETEALL, header=Header(frame_id=self.frame_id)))
    self.request_valid_angles()

  def periodic(self):
    if (self.no_paths):
      return Events.NO_PATH

    if (self.valid_angles_updated):
      # self.manager.get_logger().info(f"{self.valid_angles}")
  
      self.target_angle = self.valid_angles[self.i] % (2 * np.pi)
      self.manager.get_logger().info(f"{self.target_angle} {self.robot_pose[2]}")
      return super().periodic()
    else:
      output = Twist()
      output.angular.z = 0.0
      self.cmd_vel_publisher.publish(output)

      # repopulate valid angles
      self.request_valid_angles()

      return None
    
  def exit(self, event: Events):
    super().exit(event)
    if (event != Events.NO_PATH):
      self.completed_angles.append(self.valid_angles[self.i])

  # ===============
  # helpers
  # ===============

  def get_base_exc_angle(self):
    """
    Gets the base direction that robot should face when starting excavation. Corrects for mirrored zone in UCF
    """

    exc_center = zones.zone_to_poly(zones.exc_zone).centroid
    berm_center = zones.zone_to_poly(zones.berm_zone).centroid

    if (exc_center.x > berm_center.x): # exc is left of the berm
      angle = np.deg2rad(0.0)
    else:
      angle = np.deg2rad(180.0)

    return angle
  
  def request_valid_angles(self):
    if not self.called_costmap:
      self.called_costmap = True
      self.marker_pub.publish(Marker(action=Marker.DELETEALL, header=Header(frame_id=self.frame_id)))
  
      self.valid_angles_updated = False
      self.valid_angles = np.arange(self.min_angle, self.max_angle, self.angle_step).tolist()
      self.manager.get_logger().info("[ALIGN TO TRENCH] determining safe trench angles")
      self.costmap_client.wait_for_service()
      self.costmap_client.call_async(GetCostmap.Request()).add_done_callback(self.costmap_cb)

  def costmap_cb(self, future: Future):
    result = future.result()
    if result == None:
        return
    costmap: Costmap = result.map

    if self.robot_pose != (None, None, None) and self.robot_pose is not None:
      angles = self.valid_angles.copy()
      self.manager.get_logger().info(f"{len(angles)} {len(self.valid_angles)}")
      for angle in angles:
        _, blocked = self.evaluate_angle(costmap, angle, self.exc_approach_dist)
        if blocked:
          self.valid_angles.remove(angle)

      self.i = 0

      self.manager.get_logger().info("[ALIGN TO TRENCH] safe trench angles identified")

      if len(self.valid_angles) == 0:
        self.no_paths = True
        return

      while self.i < len(self.valid_angles) and self.valid_angles[self.i] in self.completed_angles:
        self.i += 1

      if self.i == len(self.valid_angles): # all valid angles visited
        self.manager.get_logger().info("[ALIGN TO TRENCH] all safe angles taken. incrementing approach distance")
        self.increment_exc_dist()
      else:
        self.valid_angles_updated = True

    else:
      self.manager.get_logger().info("Robot pose unknown, trying again...")

    self.called_costmap = False

  def increment_exc_dist(self):
    # update excavation approach distance
    self.exc_approach_dist += self.distance_step
    new_approach_dist = Parameter("exc_approach_dist", Parameter.Type.DOUBLE, self.exc_approach_dist)
    new_retreat_dist = Parameter("exc_retreat_dist", Parameter.Type.DOUBLE, self.exc_approach_dist + self.trenching_dist)
    self.manager.set_parameters([new_approach_dist, new_retreat_dist])

    self.completed_angles = []

    self.angle_step /= 2.0  # excavate between trenches farther away

    self.request_valid_angles() # reevaluate safe options
  
  def evaluate_angle(self, costmap: Costmap, target_angle: float, exc_approach_dist: float):
    a_x = self.robot_pose[0]
    a_y = self.robot_pose[1]
    a = shp.Point(a_x, a_y)

    b_x = (exc_approach_dist + LOOKAHEAD_DIST) * np.cos(target_angle) + self.robot_pose[0]
    b_y = (exc_approach_dist + LOOKAHEAD_DIST) * np.sin(target_angle) + self.robot_pose[1]
    b = shp.Point(b_x, b_y)

    cost, blocked = CostmapUtil.line_cost(costmap, a, b, LETHAL_COST)

    if not blocked:
      self.show_line(a, b, "good", int(np.rad2deg(target_angle)) + 360, r=0.0, g=1.0, b=0.0)
    else:
      self.show_line(a, b, "bad", int(np.rad2deg(target_angle)) + 360, r=1.0, g=0.0, b=0.0)

    return cost, blocked

  def show_line(self, p1: shp.Point, p2: shp.Point, ns, id, r=1.0, g=0.0, b=0.0, width=0.05, action=Marker.ADD):
    marker = Marker()
    marker.header.frame_id = self.frame_id
    marker.header.stamp = self.manager.get_clock().now().to_msg()

    marker.ns = ns
    marker.id = id
    marker.type = Marker.LINE_STRIP
    marker.action = action

    # Line width
    marker.scale.x = width

    # Color (RGBA)
    marker.color.r = r
    marker.color.g = g
    marker.color.b = b
    marker.color.a = 1.0

    # Identity pose
    marker.pose.orientation.w = 1.0

    p1m = Point(x = p1.x, y = p1.y)
    p2m = Point(x = p2.x, y = p2.y)

    marker.points = [p1m, p2m]

    self.manager.get_logger().info(f"Marking segment with ID {id} | P1: {p1.x:2f},{p1.y:2f} | P2: {p2.x:2f},{p2.y:2f}")
    self.marker_pub.publish(marker)