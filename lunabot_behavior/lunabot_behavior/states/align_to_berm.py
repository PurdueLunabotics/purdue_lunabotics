from lunabot_behavior.zones import berm_zone, zone_to_poly
import numpy as np
import shapely.geometry as shp

from lunabot_behavior.states.align_to_angle import AlignToAngle


class AlignToBerm(AlignToAngle):
  def __init__(self, backwards = False):
    self.backwards = backwards
    super().__init__(0)

  def periodic(self):
    if self.robot_pose is None:
      return None

    berm_center = zone_to_poly(berm_zone).centroid
    current_pos = shp.Point(self.robot_pose[0], self.robot_pose[1])
    self.target_angle = np.arctan2(berm_center.y - current_pos.y, berm_center.x - current_pos.x)

    if self.backwards:
      self.target_angle += np.pi

    self.target_angle %= 2 * np.pi

    return super().periodic()
