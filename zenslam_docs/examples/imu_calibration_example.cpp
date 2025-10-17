/**
 * @file imu_calibration_example.cpp
 * @brief Example demonstrating how to use the IMU calibration functionality
 * 
 * This example shows:
 * 1. Parsing IMU calibration from a YAML file
 * 2. Accessing individual parameters
 * 3. Using parameters in estimation algorithms
 * 
 * To build (standalone):
 *   g++ -std=c++23 -I/path/to/zenslam_core/include \
 *       -o imu_example imu_calibration_example.cpp \
 *       zenslam_core/source/imu_calibration.cpp \
 *       -lyaml-cpp -lspdlog
 * 
 * To build (with CMake):
 *   Add to zenslam_app or create a separate executable target
 */

#include <zenslam/imu_calibration.h>
#include <iostream>
#include <cmath>

int main()
{
    // Example 1: Direct parsing from file path
    try
    {
        std::cout << "Example 1: Direct parsing\n";
        std::cout << "==========================\n";
        
        auto imu_calib = zenslam::imu_calibration::parse("imu_config.yaml");
        
        std::cout << "Successfully loaded IMU calibration:\n";
        std::cout << "  Topic: " << imu_calib.rostopic << "\n";
        std::cout << "  Update rate: " << imu_calib.update_rate << " Hz\n";
        std::cout << "  Accelerometer noise density: " 
                  << imu_calib.accelerometer_noise_density << " m/s^1.5\n";
        std::cout << "  Gyroscope noise density: " 
                  << imu_calib.gyroscope_noise_density << " rad/s^0.5\n";
        std::cout << "\n";
        
        // Print all parameters using built-in method
        imu_calib.print();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error loading IMU calibration: " << e.what() << "\n";
    }

    // Example 2: Using IMU parameters in estimation
    try
    {
        std::cout << "\nExample 2: Using parameters in estimation\n";
        std::cout << "=========================================\n";
        
        auto imu_calib = zenslam::imu_calibration::parse("imu_config.yaml");
        
        // Example: Computing noise covariance matrix diagonal
        double accel_variance = imu_calib.accelerometer_noise_density * 
                               imu_calib.accelerometer_noise_density;
        double gyro_variance = imu_calib.gyroscope_noise_density * 
                              imu_calib.gyroscope_noise_density;
        
        std::cout << "Noise variances:\n";
        std::cout << "  Accelerometer: " << accel_variance << " (m/s^1.5)^2\n";
        std::cout << "  Gyroscope: " << gyro_variance << " (rad/s^0.5)^2\n";
        std::cout << "\n";
        
        // Example: Computing bias drift over time
        double time_interval = 10.0; // seconds
        double accel_drift = imu_calib.accelerometer_random_walk * std::sqrt(time_interval);
        double gyro_drift = imu_calib.gyroscope_random_walk * std::sqrt(time_interval);
        
        std::cout << "Expected bias drift over " << time_interval << " seconds:\n";
        std::cout << "  Accelerometer: " << accel_drift << " m/s^2\n";
        std::cout << "  Gyroscope: " << gyro_drift << " rad/s\n";
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << "\n";
    }
    
    // Example 3: Integration with options system
    // Note: Uncomment when using with zenslam application
    /*
    #include <zenslam/options.h>
    
    auto opts = zenslam::options::parse(argc, argv);
    auto imu_calib = zenslam::imu_calibration::parse(opts.folder.imu_calibration_file);
    */

    return 0;
}
