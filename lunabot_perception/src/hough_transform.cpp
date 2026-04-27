// I apologize in advance for how shit this code is. I... I'm trying.
#include "hough_transform.hpp"
#include <array>
#include <vector>
// #include <abs>
#include <math.h>

std::vector<std::vector<std::vector<float>>>  houghTransformRing(std::vector<std::vector<float>>* point_positions,
    std::vector<float> cx_bin_centers,  
    std::vector<float> cy_bin_centers, 
    std::vector<float> r_bin_centers, 
    float hough_epsilon){
    int cx_max = sizeof(cx_bin_centers) / sizeof(cx_bin_centers[0]);
    int cy_max = sizeof(cy_bin_centers) / sizeof(cy_bin_centers[0]);
    int r_max = sizeof(r_bin_centers) / sizeof(r_bin_centers[0]);
    float contribution;
    
    
    std::vector<float> center_r(r_max,0.0);
    std::vector<std::vector<float>> center_y(cy_max, center_r);
    std::vector<std::vector<std::vector<float>>> hough_space(cx_max, center_y);


    for(int i_cx = 0; i_cx < cx_max;i_cx++){
        for(int i_cy = 0; i_cy < cy_max;i_cy++){
            for(int i_r = 0; i_r < r_max; i_r++){
                contribution = sumPointContributions(
                    point_positions, 
                    cx_bin_centers[i_cx],
                    cy_bin_centers[i_cy], 
                    r_bin_centers[i_r], 
                    hough_epsilon
                );
                hough_space[i_cx][i_cy][i_r] = contribution;
            }
        }
    }
    return hough_space;
}

float sumPointContributions(std::vector<std::vector<float>>* point_positions,
    float cx,  
    float cy, 
    float r, 
    float hough_epsilon){
    
    float total_amplitude = 0;
    float point_x = 0;
    float point_y = 0;
    float r_point = 0;

    float point_contribution = 0;

    int point_positions_max = sizeof(point_positions) / sizeof(point_positions[0]);

    for(int i_point = 0; i_point < point_positions_max; i_point++){
        point_x = (*point_positions)[i_point][0];
        point_y = (*point_positions)[i_point][1];
        r_point = calculatePointDistance(
            cx, cy, point_x, point_y
        );
        point_contribution = applyTriangularEvaluation(
            r, 1, r_point, hough_epsilon
        );
        total_amplitude += point_contribution;
    }
    return total_amplitude;
}

float calculatePointDistance(
    float ring_cx, 
    float ring_cy, 
    float point_x, 
    float point_y
){
    float x_pos = point_x - ring_cx;
    float y_pos = point_y - ring_cy;
    // if anyone looks at this line and asks me what the hell I'm doing, I don't know either.
    float r_point = std::sqrt(x_pos*x_pos + y_pos*y_pos);
    return r_point;
}

float applyTriangularEvaluation(
    float r_point,
    float hough_epsilon,
    float ring_radius,
    float amplitude
){
    float abs_loc = 0;
    float point_contribution = 0;

    if(r_point > ring_radius-hough_epsilon && r_point < ring_radius+hough_epsilon){
        abs_loc = std::abs(r_point-ring_radius);
        point_contribution = amplitude*(hough_epsilon - abs_loc)/hough_epsilon;
    }
    return point_contribution;
}

