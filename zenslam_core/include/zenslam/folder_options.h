#pragma once

#include <filesystem>

#include <boost/program_options/options_description.hpp>

#include "zenslam/option.h"

namespace YAML
{
    class Node;
}

namespace zenslam
{
    /// File and folder path configuration
    class folder_options
    {
    public:
        using options_description = boost::program_options::options_description;

        static auto description() -> options_description;
        static auto parse_yaml(const YAML::Node& node) -> folder_options;

        // Define all options using the auto-registration macro
        // Format: ((type, name, default_value, "description"))
        ZENSLAM_DEFINE_OPTIONS
        (
            ((std::filesystem::path, root, ".", "Root folder path"))
            ((std::filesystem::path, left, "cam0", "Left camera folder path (relative to root or absolute)"))
            ((std::filesystem::path, right, "cam1", "Right camera folder path (relative to root or absolute)"))
            ((std::filesystem::path, output, "output", "Output folder for results"))
            ((std::filesystem::path, calibration_file, "camchain.yaml", "Calibration file path"))
            ((std::filesystem::path, groundtruth_file, "groundtruth.csv", "Groundtruth file path"))
            ((std::filesystem::path, imu_calibration_file, "imu_config.yaml", "IMU calibration file path"))
            ((std::filesystem::path, imu_file, "", "IMU data CSV file path"))
            ((double, timescale, 1.0, "Timescale for folder timestamps"))
        )

        void validate() const;
        void print() const;
    };
} // namespace zenslam
