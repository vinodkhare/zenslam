#include "zenslam/calibration/imu_calibration.h"

#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

auto zenslam::imu_calibration::parse(const std::filesystem::path& path) -> imu_calibration
{
    if (!std::filesystem::exists(path))
    {
        throw std::runtime_error("IMU calibration file not found: " + path.string());
    }

    auto config = YAML::LoadFile(path.string());

    imu_calibration calib;

    // Parse optional rostopic field
    if (config["rostopic"])
    {
        calib.rostopic = config["rostopic"].as<std::string>();
    }

    // Parse optional update_rate field
    if (config["update_rate"])
    {
        calib.update_rate = config["update_rate"].as<double>();
    }

    // Parse required accelerometer parameters
    if (config["accelerometer_noise_density"])
    {
        calib.accelerometer_noise_density = config["accelerometer_noise_density"].as<double>();
    }

    if (config["accelerometer_random_walk"])
    {
        calib.accelerometer_random_walk = config["accelerometer_random_walk"].as<double>();
    }

    // Parse required gyroscope parameters
    if (config["gyroscope_noise_density"])
    {
        calib.gyroscope_noise_density = config["gyroscope_noise_density"].as<double>();
    }

    if (config["gyroscope_random_walk"])
    {
        calib.gyroscope_random_walk = config["gyroscope_random_walk"].as<double>();
    }

    return calib;
}

auto zenslam::imu_calibration::print() const -> void
{
    SPDLOG_INFO("IMU Calibration:");
    if (!rostopic.empty())
    {
        SPDLOG_INFO("  ROS topic: {}", rostopic);
    }
    SPDLOG_INFO("  Update rate: {} Hz", update_rate);
    SPDLOG_INFO("  Accelerometer noise density: {} m/s^1.5", accelerometer_noise_density);
    SPDLOG_INFO("  Accelerometer random walk: {} m/s^2.5", accelerometer_random_walk);
    SPDLOG_INFO("  Gyroscope noise density: {} rad/s^0.5", gyroscope_noise_density);
    SPDLOG_INFO("  Gyroscope bias random walk: {} rad/s^1.5", gyroscope_random_walk);
}