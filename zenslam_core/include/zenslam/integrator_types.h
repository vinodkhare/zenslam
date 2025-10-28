#pragma once

#include <Eigen/Eigen>

namespace zenslam
{
    /** @brief Pre-integrated IMU measurements structure
     */
    struct integral
    {
        Eigen::Matrix3d             delta_R    = Eigen::Matrix3d::Identity();
        Eigen::Vector3d             delta_v    = Eigen::Vector3d::Zero();
        Eigen::Vector3d             delta_p    = Eigen::Vector3d::Zero();
        double                      dt         = 0.0;
        double                      dt_sq_half = 0.0;
        Eigen::Matrix<double, 9, 9> cov        = Eigen::Matrix<double, 9, 9>::Zero();
    };

    /** @brief Prior biases for IMU pre-integration
     */
    struct prior
    {
        std::vector<double> acc_bias { 0.0, 0.0, 0.0 };
        std::vector<double> gyr_bias { 0.0, 0.0, 0.0 };
    };
}