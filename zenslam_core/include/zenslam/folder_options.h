#pragma once

#include <filesystem>

#include <boost/program_options/options_description.hpp>

#include "zenslam/option.h"

namespace zenslam
{
    /// File and folder path configuration
    class folder_options
    {
    public:
        using options_description = boost::program_options::options_description;

        static options_description description();

        ZENSLAM_OPTION(std::filesystem::path, root, ".", "Root folder path");
        ZENSLAM_OPTION(std::filesystem::path, left, "cam0", "Left camera folder path (relative to root or absolute)");
        ZENSLAM_OPTION(std::filesystem::path, right, "cam1", "Right camera folder path (relative to root or absolute)");
        ZENSLAM_OPTION(std::filesystem::path, output, "output", "Output folder for results");
        ZENSLAM_OPTION(double, timescale, 1.0, "Timescale for folder timestamps");
        ZENSLAM_OPTION(std::filesystem::path, calibration_file, "camchain.yaml", "Calibration file path");
        ZENSLAM_OPTION(std::filesystem::path, groundtruth_file, "groundtruth.csv", "Groundtruth file path");
        ZENSLAM_OPTION(std::filesystem::path, imu_calibration_file, "imu_config.yaml", "IMU calibration file path");
        ZENSLAM_OPTION(std::filesystem::path, imu_file, "", "IMU data CSV file path");

        void validate() const;
        void print() const;
    };

} // namespace zenslam