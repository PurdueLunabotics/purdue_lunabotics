#define hough_transform

std::vector<std::vector<std::vector<float>>> houghTransformRing (
    std::vector<std::vector<float>>* point_positions,
    std::vector<float> cx_bin_centers,  
    std::vector<float> cy_bin_centers, 
    std::vector<float> r_bin_centers, 
    float hough_epsilon);

float sumPointContributions(
    std::vector<std::vector<float>>* point_positions,
    float cx, 
    float cy, 
    float r, 
    float hough_epsilon);

float calculatePointDistance(
    float ring_cx,
    float ring_cy,
    float point_x,
    float point_y
);

float applyTriangularEvaluation(
    float r_point,
    float hough_epsilon,
    float ring_radius,
    float amplitude
);
