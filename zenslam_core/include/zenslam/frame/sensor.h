#pragma once

#include <array>
#include <cmath>

#include <opencv2/core/mat.hpp>

namespace zenslam::frame
{
    struct sensor
    {
        static size_t count;

        size_t                 index     = { };
        double                 timestamp = { std::nan("nan") };
        std::array<cv::Mat, 2> images    = { };
    };
}
