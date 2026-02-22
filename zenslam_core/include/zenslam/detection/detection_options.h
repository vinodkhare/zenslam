#pragma once

#include <opencv2/core/types.hpp>

#include "descriptor_type.h"
#include "detection_algorithm.h"
#include "feature_type.h"

namespace zenslam
{
    struct detection_options
    {
        bool                clahe_enabled         = false;
        bool                stereo_rectify        = false;
        cv::Size            cell_size             = cv::Size(16, 16);
        int                 fast_threshold        = 10;
        double              keyline_max_length    = 128; // Maximum length of a keyline in pixels
        feature_type        feature_detector      = feature_type::FAST;
        descriptor_type     descriptor            = descriptor_type::ORB;
        detection_algorithm algorithm             = detection_algorithm::GRID;
    };
}
