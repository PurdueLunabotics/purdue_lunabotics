import numpy as np
import math

def hough_transform_ring (point_positions, cx_bin_centers,  cy_bin_centers, r_bin_centers, hough_epsilon: np.float32):
    i_cx = 0
    i_cy = 0
    i_r = 0
    cx_max = cx_bin_centers.shape[0]
    cy_max = cy_bin_centers.shape[0]
    r_max = r_bin_centers.shape[0]
    contribution : np.float32 
    houghSpace : np.ndarray[np.float32] = np.zeros(
        [cx_max, cy_max, r_max],
        dtype=np.float32
    )
    for i_cx in range(cx_max):
        for i_cy in range(cy_max):
            for i_r in range(r_max):
                contribution = sum_point_contributions(
                    point_positions, 
                    cx_bin_centers[i_cx],
                    cy_bin_centers[i_cy], 
                    r_bin_centers[i_r], 
                    hough_epsilon
                )
                houghSpace[i_cx, i_cy, i_r] = contribution
    return houghSpace


def sum_point_contributions(
    point_positions,
    cx : np.float32, 
    cy : np.float32, 
    r : np.float32, 
    hough_epsilon : np.float32
):
    total_amplitude : np.float32 = 0
    point_x : np.float32
    point_y : np.float32
    r_point : np.float32
    point_contribution : np.float32
    point_positions_max = point_positions.shape[0]
    for i_point in range(point_positions_max):
        point_x = point_positions[i_point, 0]
        point_y = point_positions[i_point, 1]
        r_point = calculate_point_distance(
            ring_cx=cx, ring_cy=cy,
            point_x=point_x, point_y=point_y)
        point_contribution = apply_triangular_evaluation(
            ring_radius=r, amplitude=5,
            r_point=r_point, hough_epsilon=hough_epsilon)
        total_amplitude += point_contribution
    return total_amplitude


def calculate_point_distance(
    ring_cx: np.float32,
    ring_cy: np.float32,
    point_x: np.float32,
    point_y : np.float32
):
    x_pos : np.float32 = point_x-ring_cx
    y_pos : np.float32 = point_y-ring_cy
    r_point : np.float32 = math.sqrt(x_pos**2 + y_pos**2)
    return r_point

def apply_triangular_evaluation(
    r_point : np.float32,
    hough_epsilon : np.float32,
    ring_radius : np.float32,
    amplitude : np.float32
):
    abs_loc : np.float32
    point_contribution : np.float32
    if (
        r_point > ring_radius-hough_epsilon and
        r_point < ring_radius+hough_epsilon
    ):
        abs_loc = abs(r_point-ring_radius)
        point_contribution = amplitude*(
            hough_epsilon-abs_loc)/(hough_epsilon)
    else:
        point_contribution = 0
    return point_contribution