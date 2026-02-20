#pragma once

#include <filesystem>
#include <string>

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>

#include "zenslam/option.h"
#include "zenslam/options_base.h"

namespace YAML
{
    class Node;
}

namespace zenslam
{
    /// File and folder path configuration
    class folder_options : public options_base<folder_options, "folder options", "folder">
    {
    public:
        // Inherited from options_base:
        // static constexpr auto name() - returns "folder options"
        // static constexpr auto prefix() - returns "folder"
        // static auto description() -> boost::program_options::options_description;
        // static auto parse_yaml(const YAML::Node& node) -> folder_options;
        // static void parse_cli(folder_options& options, const std::map<std::string, boost::program_options::basic_option<char>>& options_map, const boost::program_options::variables_map& vm);
        // void print() const;

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
    };
} // namespace zenslam
