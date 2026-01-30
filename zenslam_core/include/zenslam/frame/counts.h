#pragma once

#include <cstddef>
#include <string>

namespace zenslam::frame
{
    struct counts
    {
        size_t keypoints_l                  = { };
        size_t keypoints_r                  = { };
        size_t keypoints_l_tracked          = { };
        size_t keypoints_r_tracked          = { };
        size_t keypoints_l_new              = { };
        size_t keypoints_r_new              = { };
        size_t keypoints_total              = { };
        size_t matches                      = { };
        size_t matches_triangulated         = { };
        size_t correspondences_3d3d         = { };
        size_t correspondences_3d2d         = { };
        size_t correspondences_2d2d         = { };
        size_t correspondences_3d3d_inliers = { };
        size_t correspondences_3d2d_inliers = { };
        size_t correspondences_2d2d_inliers = { };
        size_t points                       = { };
        size_t lines                        = { };

        // Quality metrics
        double klt_error_mean               = { };
        double klt_error_std                = { };
        double klt_success_rate             = { };
        double match_distance_mean          = { };
        double match_distance_std           = { };
        double epipolar_error_mean          = { };
        size_t fundamental_inliers          = { };
        double response_mean_l              = { };
        double response_mean_r              = { };
        double response_std_l               = { };
        double response_std_r               = { };
        double grid_occupancy_l             = { };
        double grid_occupancy_r             = { };
        double track_age_mean               = { };
        double track_age_max                = { };
        
        // Configuration tracking
        std::string detector_type           = { };
        std::string descriptor_type         = { };
        std::string matcher_type            = { };

        auto print() const -> void;
    };
}