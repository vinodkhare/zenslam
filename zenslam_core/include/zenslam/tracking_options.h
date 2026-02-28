#pragma once

#include <opencv2/core/types.hpp>

namespace zenslam
{
    struct tracking_options
    {
        cv::Size klt_window_size         = cv::Size(31, 31);
        int      klt_max_level           = 3;
        double   klt_threshold           = 1.0;
        double   klt_min_tracked_ratio   = 0.6;
        double   landmark_match_distance = 32.0; // Descriptor distance threshold for matching keypoints to landmarks
        double   landmark_match_radius   = 50.0; // Radius in meters for landmark matching around the camera
        bool     use_keylines            = true; // Enable keyline tracking and triangulation
        bool     filter_epipolar         = true;
        double   epipolar_threshold      = 1.0; // Threshold in pixels for epipolar filtering of keypoint matches
    };
}
