import numpy as np
from hough_transform import hough_transform_ring


def get_bin_centers(
    c_x, c_y, r, uncertainty
):
    cx_bin_centers = np.linspace(
        start=c_x - 0.5*uncertainty,
        stop=c_x + 0.5*uncertainty,
        num=11)
    cy_bin_centers = np.linspace(
        start=c_y - 0.5*uncertainty,
        stop=c_y + 0.5*uncertainty,
        num=11)
    r_bin_centers = np.linspace(
        start=r - 0.25,
        stop=r + 0.25,
        num=11)
    return cx_bin_centers, cy_bin_centers, r_bin_centers


def interpretHoughSpace(
    houghSpace
):
    indices_of_maximum_value = np.unravel_index(
        np.argmax(houghSpace), shape=houghSpace.shape)
    return indices_of_maximum_value


def advanced_guess_with_hough(
    guessed_cx, guessed_cy, guessed_r, point_cloud,
    uncertainty, epsilon
):
    cx_bin_centers, cy_bin_centers, r_bin_centers = (
        get_bin_centers(guessed_cx, guessed_cy, guessed_r, uncertainty)
    )
    epsilon = np.float32(epsilon)
    cx_bin_centers = cx_bin_centers.astype(np.float32)
    cy_bin_centers = cy_bin_centers.astype(np.float32)
    r_bin_centers = r_bin_centers.astype(np.float32)
    point_positions = np.array([s.astype(np.float32) for s in point_cloud])
    houghSpace = hough_transform_ring(
        point_positions, cx_bin_centers,
        cy_bin_centers, r_bin_centers, epsilon
    )
    location_for_maxima = interpretHoughSpace(houghSpace)
    hough_cx_idx = int(location_for_maxima[0])
    hough_cy_idx = int(location_for_maxima[1])
    hough_r_idx = int(location_for_maxima[2])
    hough_cx = cx_bin_centers[hough_cx_idx]
    hough_cy = cy_bin_centers[hough_cy_idx]
    hough_r = r_bin_centers[hough_r_idx]
    return hough_cx, hough_cy, hough_r


def compare_old_new(previous_muon_features, muon_features):
    old_cx = previous_muon_features['cx']
    old_cy = previous_muon_features['cy']
    old_r = previous_muon_features['r']
    cx = muon_features['cx']
    cy = muon_features['cy']
    r = muon_features['r']
    d_cx = abs(old_cx)-abs(cx)
    d_cy = abs(old_cy)-abs(cy)
    d_r = abs(old_r)-abs(r)
    return d_cx, d_cy, d_r


def hough_pointcloud(
    guessed_cx, guessed_cy, guessed_r,
    point_cloud, uncertainty, epsilon,
    max_iter=20
):
    ring_features = {}
    ring_features['cx'] = guessed_cx
    ring_features['cy'] = guessed_cy
    ring_features['r'] = guessed_r
    i = 0
    while i < max_iter:
        i += 1
        previous_ring_features = ring_features.copy()
        hough_cx, hough_cy, hough_r = advanced_guess_with_hough(
            ring_features['cx'], ring_features['cy'],
            ring_features['r'], point_cloud, uncertainty, epsilon)
        ring_features['cx'] = hough_cx
        ring_features['cy'] = hough_cy
        ring_features['r'] = hough_r
        d_cx, d_cy, d_r = compare_old_new(
            previous_ring_features, ring_features
        )
        uncertainty /= 2
        if (
            d_cx <= np.deg2rad(0.05) and
            d_cy <= np.deg2rad(0.05) and
            d_r <= np.deg2rad(0.03) and
            i >= 6
        ):
            break
    return hough_cx, hough_cy, hough_r