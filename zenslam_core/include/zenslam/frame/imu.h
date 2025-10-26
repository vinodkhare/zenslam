#pragma once

#include <cmath>

#include <opencv2/core/matx.hpp>

namespace zenslam::frame
{
    struct imu
    {
        double    timestamp    = { std::nan("nan") }; // timestamp in seconds
        cv::Vec3d acc = { 0.0, 0.0, 0.0 };   // linear acceleration (m/s²)
        cv::Vec3d gyr     = { 0.0, 0.0, 0.0 };   // angular velocity (rad/s)
    };
}
