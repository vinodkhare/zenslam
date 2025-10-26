/**
 * @file imu_pose_prediction_example.cpp
 * @brief Example demonstrating IMU-based pose prediction using pre-integration
 * 
 * This example shows how to:
 * 1. Pre-integrate IMU measurements over a time interval
 * 2. Predict the next pose given current pose and velocity
 * 3. Use predictions as initial guesses for visual odometry
 */

#include <iostream>
#include <vector>
#include <opencv2/core/affine.hpp>
#include "zenslam/preint.h"
#include "zenslam/imu_calibration.h"
#include "zenslam/frame/sensor.h"
#include "zenslam/utils_slam.h"

int main()
{
    // ========================================
    // 1. Setup IMU Calibration
    // ========================================
    zenslam::imu_calibration imu_calib;
    imu_calib.accelerometer_noise_density = 0.01;  // m/s^2/sqrt(Hz)
    imu_calib.gyroscope_noise_density = 0.001;     // rad/s/sqrt(Hz)
    
    // ========================================
    // 2. Create Synthetic IMU Data
    // ========================================
    std::vector<zenslam::frame::imu_measurement> imu_data;
    
    // Simulate constant angular velocity and linear acceleration
    const double dt_imu = 0.005;  // 200 Hz IMU
    const double duration = 0.1;  // 100ms interval
    const int num_samples = static_cast<int>(duration / dt_imu);
    
    for (int i = 0; i < num_samples; ++i)
    {
        zenslam::frame::imu_measurement meas;
        meas.timestamp = i * dt_imu;
        
        // Constant forward acceleration + gravity compensation
        meas.alpha_x = 0.0;
        meas.alpha_y = 0.0;
        meas.alpha_z = 9.81 + 0.5;  // Upward accel of 0.5 m/s^2
        
        // Slight rotation around Z-axis
        meas.omega_x = 0.0;
        meas.omega_y = 0.0;
        meas.omega_z = 0.1;  // 0.1 rad/s rotation
        
        imu_data.push_back(meas);
    }
    
    // ========================================
    // 3. Create Preintegrator and Integrate
    // ========================================
    zenslam::integrator imu_preint(imu_calib, zenslam::integrator::method::ugpm);
    
    // Configure for optimal performance
    imu_preint.set_overlap_factor(8);
    imu_preint.set_state_frequency(50.0);
    imu_preint.set_correlate(true);
    
    const double start_t = imu_data.front().timestamp;
    const double end_t = imu_data.back().timestamp;
    
    auto preint_result = imu_preint.integrate(imu_data, start_t, end_t);
    
    if (!preint_result.has_value())
    {
        std::cerr << "IMU pre-integration failed!" << std::endl;
        return 1;
    }
    
    const auto& preint_meas = preint_result.value();
    
    std::cout << "=== IMU Pre-integration Results ===" << std::endl;
    std::cout << "Time interval: " << preint_meas.dt << " seconds" << std::endl;
    std::cout << "Delta rotation (SO3):" << std::endl;
    for (int i = 0; i < 3; ++i)
    {
        std::cout << "  [" << preint_meas.delta_R(i, 0) << ", "
                  << preint_meas.delta_R(i, 1) << ", "
                  << preint_meas.delta_R(i, 2) << "]" << std::endl;
    }
    std::cout << "Delta velocity: ["
              << preint_meas.delta_v(0) << ", "
              << preint_meas.delta_v(1) << ", "
              << preint_meas.delta_v(2) << "] m/s" << std::endl;
    std::cout << "Delta position: ["
              << preint_meas.delta_p(0) << ", "
              << preint_meas.delta_p(1) << ", "
              << preint_meas.delta_p(2) << "] m" << std::endl;
    
    // ========================================
    // 4. Predict Next Pose
    // ========================================
    
    // Initial state (world frame)
    cv::Affine3d pose_0 = cv::Affine3d::Identity();
    pose_0.translation(cv::Vec3d(0.0, 0.0, 1.0));  // Start at 1m height
    
    cv::Vec3d velocity_0(0.0, 0.0, 0.0);  // Initially at rest
    
    // Predict pose at t1
    cv::Affine3d predicted_pose = zenslam::integrator::predict_pose(
        pose_0,
        velocity_0,
        preint_meas
    );
    
    // Predict velocity at t1
    cv::Vec3d predicted_velocity = zenslam::integrator::predict_velocity(
        pose_0,
        velocity_0,
        preint_meas
    );
    
    std::cout << "\n=== Pose Prediction ===" << std::endl;
    std::cout << "Initial pose:" << std::endl;
    std::cout << "  Position: [" << pose_0.translation()[0] << ", "
              << pose_0.translation()[1] << ", "
              << pose_0.translation()[2] << "]" << std::endl;
    std::cout << "  Velocity: [" << velocity_0[0] << ", "
              << velocity_0[1] << ", "
              << velocity_0[2] << "] m/s" << std::endl;
    
    std::cout << "\nPredicted pose (after " << preint_meas.dt << "s):" << std::endl;
    std::cout << "  Position: [" << predicted_pose.translation()[0] << ", "
              << predicted_pose.translation()[1] << ", "
              << predicted_pose.translation()[2] << "]" << std::endl;
    std::cout << "  Velocity: [" << predicted_velocity[0] << ", "
              << predicted_velocity[1] << ", "
              << predicted_velocity[2] << "] m/s" << std::endl;
    
    // ========================================
    // 5. Integration with Visual Tracking
    // ========================================
    
    std::cout << "\n=== Integration Example ===" << std::endl;
    std::cout << "Use case: Visual-Inertial SLAM" << std::endl;
    std::cout << "1. Use predicted_pose as initial guess for feature matching" << std::endl;
    std::cout << "2. Predict 2D feature positions: p_2d = K * [R|t] * p_3d" << std::endl;
    std::cout << "3. Search for features in predicted regions (smaller search window)" << std::endl;
    std::cout << "4. Refine pose using visual measurements" << std::endl;
    std::cout << "5. Update velocity estimate using refined pose" << std::endl;
    
    // ========================================
    // 6. Covariance Information
    // ========================================
    
    std::cout << "\n=== Uncertainty ===" << std::endl;
    std::cout << "Covariance diagonal (rotation, velocity, position):" << std::endl;
    for (int i = 0; i < 9; ++i)
    {
        std::cout << "  " << i << ": " << preint_meas.cov(i, i) << std::endl;
    }
    
    std::cout << "\nNote: Covariance can be used for:" << std::endl;
    std::cout << "  - Weighting visual-inertial fusion" << std::endl;
    std::cout << "  - Outlier rejection thresholds" << std::endl;
    std::cout << "  - Kalman filter/optimization information matrices" << std::endl;
    
    return 0;
}
