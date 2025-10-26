#pragma once

#include <array>
#include <cmath>
#include <vector>

#include <opencv2/core/mat.hpp>

namespace zenslam::frame
{
    struct imu_measurement
    {
        double timestamp = { std::nan("nan") };  // timestamp in seconds
        double omega_x   = { 0.0 };              // angular velocity x (rad/s)
        double omega_y   = { 0.0 };              // angular velocity y (rad/s)
        double omega_z   = { 0.0 };              // angular velocity z (rad/s)
        double alpha_x   = { 0.0 };              // linear acceleration x (m/s²)
        double alpha_y   = { 0.0 };              // linear acceleration y (m/s²)
        double alpha_z   = { 0.0 };              // linear acceleration z (m/s²)
    };

    struct sensor
    {
        static size_t count;

        size_t                          index          = { };
        double                          timestamp      = { std::nan("nan") };
        std::array<cv::Mat, 2>          images         = { };
        std::vector<imu_measurement>    imu_data       = { };  // IMU measurements between frames
    };
}
