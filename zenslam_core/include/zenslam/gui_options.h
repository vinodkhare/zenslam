#pragma once

#include <opencv2/core/types.hpp>

namespace zenslam
{
    struct gui_options
    {
        bool       show_keypoints       = true;
        bool       show_keylines        = true;
        cv::Scalar keyline_single_color = cv::Scalar(0, 255, 255); // BGR
        cv::Scalar keyline_match_color  = cv::Scalar(0, 255, 0);   // BGR
        int        keyline_thickness    = 1;
        double     point_cloud_opacity  = 1.0;
        float      point_size           = 4.0f;
    };
}
