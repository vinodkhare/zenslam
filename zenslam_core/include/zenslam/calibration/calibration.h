#pragma once

#include "camera_calibration.h"
#include "imu_calibration.h"

namespace zenslam
{
    struct calibration
    {
        std::array<camera_calibration, 2> cameras            = { };
        std::array<cv::Matx33d, 2>        camera_matrix      = { };
        std::array<cv::Matx33d, 2>        fundamental_matrix = { };
        std::array<cv::Matx34d, 2>        projection_matrix  = { };
        std::array<cv::Mat, 2>            map_x              = { };
        std::array<cv::Mat, 2>            map_y              = { };
        imu_calibration                   imu                = { };

        static auto parse
        (
            const std::filesystem::path& camera_calib_path,
            const std::filesystem::path& imu_calib_path,
            bool                         stereo_rectify
        ) -> calibration;

        void print() const;
    };
}