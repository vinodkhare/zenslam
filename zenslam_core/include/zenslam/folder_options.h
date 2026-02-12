#pragma once

#include <filesystem>

#include <boost/program_options/options_description.hpp>

namespace zenslam
{
    /// File and folder path configuration
    class folder_options
    {
    public:
        using options_description = boost::program_options::options_description;

        static options_description description();

        std::filesystem::path root                 = { "." };
        std::filesystem::path left                 = { "cam0" };
        std::filesystem::path right                = { "cam1" };
        std::filesystem::path output               = { "output" };
        double                timescale            = { 1.0 };
        std::filesystem::path calibration_file     = { "camchain.yaml" };
        std::filesystem::path groundtruth_file     = { "groundtruth.csv" };
        std::filesystem::path imu_calibration_file = { "imu_config.yaml" };
        std::filesystem::path imu_file             = { "" };

        void validate() const;
        void print() const;
    };

} // namespace zenslam