import numpy as np


def rotate(pts, rad, rotate_pt):
    """Applies a 2d rotation to an array of points
    Args:
        points (np.array): array of dim 2xN, where N is the number of points
        rad ([type]): [description]

    Returns:
        [type]: [description]
    """
    pts -= rotate_pt
    c, s = np.cos(rad), np.sin(rad)
    j = np.array([[c, s], [-s, c]])
    pts = j @ pts
    pts += rotate_pt

    return pts


def __CollisionCheck(self, map, origin, node):
    """
    Checks whether a given configuration is valid. (collides with obstacles)
    """

    pos = node.state[0:2] - origin[0:2]
    translated_robot = self.footprint + pos  # maps to origin in 2d
    # robot bounding box = [x_min,y_min, x_max,y_max]
    r_bnds = np.hstack(
        (np.min(translated_robot, axis=0), np.max(translated_robot, axis=0))
    )

    occupied_i = np.array(np.nonzero(self.grid > 0.5))  # flat coords of obstacles
    occupied_i = np.vstack(
        (occupied_i / self.robot_w, occupied_i % self.robot_w)
    )  # xy coords
    occupied_i *= self.resolution  # maps to real-world coordinates
    occupied_i -= self.origin[0:2].reshape(2, 1)  # maps to map frame in 2d
    occupied_i = rotate(
        occupied_i, -node.state[-1], pos.reshape(2, 1)
    )  # rotate points onto robot to check collisions at angle

    is_colliding = np.any(
        (occupied_i >= r_bnds[:2].reshape(2, 1))
        & (occupied_i <= r_bnds[2:].reshape(2, 1))
    )  # colliding
    return not is_colliding
