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

    inline YAML::Emitter& operator<<(YAML::Emitter& emitter, const folder_options& folder_options)
    {
        emitter << YAML::BeginMap;
        emitter << YAML::Key << "root" << YAML::Value << folder_options.root.string();
        emitter << YAML::Key << "left" << YAML::Value << folder_options.left.string();
        emitter << YAML::Key << "right" << YAML::Value << folder_options.right.string();
        emitter << YAML::Key << "output" << YAML::Value << folder_options.output.string();
        emitter << YAML::Key << "calibration_file" << YAML::Value << folder_options.calibration_file.string();
        emitter << YAML::Key << "groundtruth_file" << YAML::Value << folder_options.groundtruth_file.string();
        emitter << YAML::Key << "imu_calibration_file" << YAML::Value << folder_options.imu_calibration_file.string();
        emitter << YAML::Key << "imu_file" << YAML::Value << folder_options.imu_file.string();
        emitter << YAML::Key << "timescale" << YAML::Value << YAML::Precision(3) << folder_options.timescale;
        emitter << YAML::Key << "skip_frames" << YAML::Value << folder_options.skip_frames;
        emitter << YAML::Key << "take_frames" << YAML::Value << folder_options.take_frames;
        emitter << YAML::EndMap;
        emitter << YAML::Newline;
        emitter << YAML::Newline;

        return emitter;
    }
}
