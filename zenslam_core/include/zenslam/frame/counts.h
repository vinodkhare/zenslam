#pragma once

#include <string>

namespace zenslam::frame
{
    /** Generic feature counts for any feature type (points or lines) */
    struct feature_counts
    {
        size_t features_l           = { };  // Features detected in left camera
        size_t features_r           = { };  // Features detected in right camera
        size_t features_l_tracked   = { };  // Features tracked in left camera
        size_t features_r_tracked   = { };  // Features tracked in right camera
        size_t features_l_new       = { };  // New features detected in left camera
        size_t features_r_new       = { };  // New features detected in right camera
        size_t features_total       = { };  // Total features (left + right)
        size_t matches_stereo       = { };  // Matches between left and right cameras
        size_t triangulated_3d      = { };  // Successfully triangulated 3D features
    };

    struct counts
    {
        // Feature statistics
        feature_counts points       = { };  // Point feature statistics
        feature_counts lines        = { };  // Line feature statistics
        
        size_t correspondences_3d3d         = { };
        size_t correspondences_3d2d         = { };
        size_t correspondences_2d2d         = { };
        size_t correspondences_3d3d_inliers = { };
        size_t correspondences_3d2d_inliers = { };
        size_t correspondences_2d2d_inliers = { };
        size_t map_points                   = { };  // Total 3D points in map
        size_t map_lines                    = { };  // Total 3D lines in map

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