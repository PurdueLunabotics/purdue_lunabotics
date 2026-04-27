using namespace std;
#include "hough_transform.hpp"
#include <vector>
#include <algorithm>

vector<float> linspace(float start, float stop, int num){
        vector<float> result(num);
        float step = (stop - start) / (num - 1);
        for(int i = 0; i < num; i++){
            result[i] = start + i * step;
        }
        return result;
    }

vector<vector<float>> getBinCenters(
    float c_x, float c_y, float r, float uncertainty_pos, float uncertainty_r
){
    vector<float> cx_bin_centers = linspace(
        c_x - 0.5*uncertainty_pos,
        c_x + 0.5*uncertainty_pos,
        11);
    vector<float> cy_bin_centers = linspace(
        c_y - 0.5*uncertainty_pos,
        c_y + 0.5*uncertainty_pos,
        11);
    vector<float> r_bin_centers = linspace(
        r - 0.25*uncertainty_r,
        r + 0.25*uncertainty_r,
        11);

    return vector<vector<float>>{cx_bin_centers, cy_bin_centers, r_bin_centers};
}

vector<int> maxIndexForEachDim(vector<vector<vector<float>>> hough_space, int dim_checked){
    
}


// vector<vector<int>> interpretHoughSpace(
//     vector<vector<vector<float>>>  hough_space
// ){
//     vector<int> index_in_dim = {1,0};
//     vector<vector<int>>indices_of_maximum_value = {3, index_in_dim};

    
//     for (int i = 0; i < 3; i++){
//         vector<int> index_in_dim = distance(hough_space[i].begin(),*max_element(hough_space[i].begin(), hough_space[i].end()));
//     }
    
//     return indices_of_maximum_value
// }
    


// def advanced_guess_with_hough(
//     guessed_cx, guessed_cy, guessed_r, point_cloud,
//     uncertainty_pos, uncertainty_r, epsilon
// ):
//     cx_bin_centers, cy_bin_centers, r_bin_centers = (
//         get_bin_centers(guessed_cx, guessed_cy, guessed_r, uncertainty_pos, uncertainty_r)
//     )
//     epsilon = np.float32(epsilon)
//     cx_bin_centers = cx_bin_centers.astype(np.float32)
//     cy_bin_centers = cy_bin_centers.astype(np.float32)
//     r_bin_centers = r_bin_centers.astype(np.float32)
//     point_positions = np.array([s.astype(np.float32) for s in point_cloud])
//     houghSpace = hough_transform_ring(
//         point_positions, cx_bin_centers,
//         cy_bin_centers, r_bin_centers, epsilon
//     )
//     location_for_maxima = interpretHoughSpace(houghSpace)
//     hough_cx_idx = int(location_for_maxima[0])
//     hough_cy_idx = int(location_for_maxima[1])
//     hough_r_idx = int(location_for_maxima[2])
//     hough_cx = cx_bin_centers[hough_cx_idx]
//     hough_cy = cy_bin_centers[hough_cy_idx]
//     hough_r = r_bin_centers[hough_r_idx]
//     return hough_cx, hough_cy, hough_r


// float32[] compare_old_new(previous_muon_features, muon_features){
//     return [d_cx, d_cy, d_r]
// }
    


// def hough_pointcloud(
//     guessed_cx, guessed_cy, guessed_r,
//     point_cloud, uncertainty_pos, uncertainty_r, epsilon,
//     max_iter=20
// ):

//     return hough_cx, hough_cy, hough_r