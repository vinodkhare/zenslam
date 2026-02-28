#include "zenslam/yaml_emitters.h"

YAML::Emitter& zenslam::operator<<(YAML::Emitter& emitter, const folder_options& folder_options)
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