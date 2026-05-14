import numpy as np
import math

def hough_transform_ring (point_positions, cx_bin_centers,  cy_bin_centers, r_bin_centers, hough_epsilon: np.float32):
    n_cx = len(cx_bin_centers)
    n_cy = len(cy_bin_centers)
    n_r = len(r_bin_centers)

    hough_space = np.empty(
        (n_cx, n_cy, n_r),
        dtype=np.float32
    )

    px = point_positions[:, 0]
    py = point_positions[:, 1]

    for i_cx, cx in enumerate(cx_bin_centers):
        dx2 = (px - cx) ** 2

        for i_cy, cy in enumerate(cy_bin_centers):

            # distances for ALL points
            distances = np.sqrt(
                dx2 + (py - cy) ** 2
            )

            # broadcast against ALL radii
            abs_loc = np.abs(
                distances[:, None] - r_bin_centers[None, :]
            )

            contributions = np.clip(
                5.0 * (hough_epsilon - abs_loc) / hough_epsilon,
                0.0,
                None
            )

            # sum over points
            hough_space[i_cx, i_cy, :] = contributions.sum(axis=0)

    return hough_space
