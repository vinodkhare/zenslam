#pragma once

#include <array>
#include <cmath>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "imu.h"

namespace zenslam::frame
{
    struct sensor
    {
        static size_t count;

        size_t                 index     = { };
        double                 timestamp = { std::nan("nan") };
        std::array<cv::Mat, 2> images    = { };
        std::vector<imu>       imu_data  = { }; // IMU measurements between frames
    };
}
