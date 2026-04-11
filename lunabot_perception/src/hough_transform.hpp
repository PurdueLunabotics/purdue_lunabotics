#ifndef HOUGH_TRANSFORM_HPP
#define HOUGH_TRANSFORM_HPP

std::array<float32> HoughTransform::houghTransformRing (
    std::array<float32> point_positions,
    std::array<float32> cx_bin_centers,  
    std::array<float32> cy_bin_centers, 
    std::array<float32> r_bin_centers, 
    float32 hough_epsilon);

float32 HoughTransform::sumPointContributions(
    std::array<float32> point_positions,
    float32 cx, 
    float32 cy, 
    float32 r, 
    float32 hough_epsilon);

float32 HoughTransform::calculate_point_distance(
    float32 ring_cx,
    float32 ring_cy,
    float32 point_x,
    float32 point_y
);

float32 HoughTransform::apply_triangular_evaluation(
    float32 r_point,
    float32 hough_epsilon,
    float32 ring_radius,
    float32 amplitude
);

#endif