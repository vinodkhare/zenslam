#pragma once

#include <filesystem>

#include <yaml-cpp/yaml.h>

namespace zenslam
{
    struct folder_options
    {
        std::filesystem::path root                 = ".";
        std::filesystem::path left                 = "cam0";
        std::filesystem::path right                = "cam1";
        std::filesystem::path output               = "output";
        std::filesystem::path calibration_file     = "calibration.yaml";
        std::filesystem::path groundtruth_file     = "groundtruth.csv";
        std::filesystem::path imu_calibration_file = "imu_config.yaml";
        std::filesystem::path imu_file             = "";
        double                timescale            = 1.0;
        size_t                skip_frames          = 0; // skip first N frames
        size_t                take_frames          = 0; // take only first N frames (after skipping), 0 for unlimited, minimum 1 frame must be taken
    };
}
