#include 'hough_transform.hpp'

std::array<float32> HoughTransform::houghTransformRing(std::array<float32> point_positions,
    std::vector<float32> cx_bin_centers,  
    std::vector<float32> cy_bin_centers, 
    std::vector<float32> r_bin_centers, 
    float32 hough_epsilon){
    int i_cx = 0;
    int i_cy = 0;
    int i_r = 0;
    int cx_max = sizeof(cx_bin_centers) / sizeof(cx_bin_centers[0]);
    int cy_max = sizeof(cy_bin_centers) / sizeof(cy_bin_centers[0]);
    int r_max = sizeof(r_bin_centers) / sizeof(r_bin_centers[0]);
    float32 contribution;

    
    
    std::array<float32, > houghSpace = 

}