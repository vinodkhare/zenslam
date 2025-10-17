#pragma once

#include <filesystem>
#include <string>

namespace zenslam
{
    /**
     * @brief IMU calibration parameters structure
     * 
     * This structure holds IMU noise and bias parameters typically used for
     * sensor fusion and state estimation. The parameters are commonly obtained
     * from Allan variance analysis.
     * 
     * Example usage:
     * @code
     * auto imu_calib = zenslam::imu_calibration::parse("/path/to/imu_config.yaml");
     * std::cout << "Gyroscope noise density: " << imu_calib.gyroscope_noise_density << std::endl;
     * @endcode
     */
    class imu_calibration
    {
    public:
        /**
         * @brief Parse IMU calibration from a YAML file
         * 
         * Reads IMU parameters from a YAML file containing the following fields:
         * - rostopic: IMU topic name (optional)
         * - update_rate: IMU update rate in Hz (optional)
         * - accelerometer_noise_density: Accelerometer white noise (m/s^1.5)
         * - accelerometer_random_walk: Accelerometer bias random walk (m/s^2.5)
         * - gyroscope_noise_density: Gyroscope white noise (rad/s^0.5)
         * - gyroscope_random_walk: Gyroscope bias random walk (rad/s^1.5)
         * 
         * @param path Path to the IMU calibration YAML file
         * @return imu_calibration object with parsed parameters
         * @throws std::runtime_error if file not found or parsing fails
         * 
         * @note If optional fields are not present in the file, default values are used
         */
        static imu_calibration parse(const std::filesystem::path &path);

        /// IMU topic name (e.g., "/imu0")
        std::string rostopic = { };

        /// IMU update rate in Hz
        double update_rate = 200.0;

        /// Accelerometer white noise density (m/s^1.5)
        double accelerometer_noise_density = 0.0;

        /// Accelerometer bias random walk (m/s^2.5)
        double accelerometer_random_walk = 0.0;

        /// Gyroscope white noise density (rad/s^0.5)
        double gyroscope_noise_density = 0.0;

        /// Gyroscope bias random walk (rad/s^1.5)
        double gyroscope_random_walk = 0.0;

        /**
         * @brief Print IMU calibration parameters to log
         * 
         * Outputs all IMU calibration parameters using spdlog
         */
        auto print() const -> void;
    };
} // namespace zenslam