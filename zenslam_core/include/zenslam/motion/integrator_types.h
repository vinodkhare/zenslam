#pragma once

#include <opencv2/core/types.hpp>

namespace zenslam
{
    /** @brief Pre-integrated IMU measurements structure
     */
    struct integral
    {
        cv::Matx33d            delta_R    = cv::Matx33d::eye();
        cv::Vec3d              delta_v    = cv::Vec3d(0, 0, 0);
        cv::Vec3d              delta_p    = cv::Vec3d(0, 0, 0);
        double                 dt         = 0.0;
        double                 dt_sq_half = 0.0;
        cv::Matx<double, 9, 9> cov        = cv::Matx<double, 9, 9>::zeros();
    };

    /** @brief Prior biases for IMU pre-integration
     */
    struct prior
    {
        std::vector<double> acc_bias { 0.0, 0.0, 0.0 };
        std::vector<double> gyr_bias { 0.0, 0.0, 0.0 };
    };
}
