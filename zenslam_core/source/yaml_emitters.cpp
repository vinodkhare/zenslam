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

YAML::Emitter& zenslam::operator<<(YAML::Emitter& emitter, const gui_options& gui_options)
{
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "show_keypoints" << YAML::Value << gui_options.show_keypoints;
    emitter << YAML::Key << "show_keylines" << YAML::Value << gui_options.show_keylines;
    emitter << YAML::Key << "keyline_thickness" << YAML::Value << gui_options.keyline_thickness;
    emitter << YAML::Key << "point_cloud_opacity" << YAML::Value << YAML::Precision(3) << gui_options.point_cloud_opacity;
    emitter << YAML::Key << "point_size" << YAML::Value << gui_options.point_size;
    emitter << YAML::EndMap;
    emitter << YAML::Newline;
    emitter << YAML::Newline;

    return emitter;
}

YAML::Emitter& zenslam::operator<<(YAML::Emitter& emitter, const tracking_options& tracking_options)
{
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "klt_window_size" << YAML::Value << YAML::Flow << YAML::BeginSeq << tracking_options.klt_window_size.width << tracking_options.klt_window_size.height << YAML::EndSeq;
    emitter << YAML::Key << "klt_max_level" << YAML::Value << tracking_options.klt_max_level;
    emitter << YAML::Key << "klt_threshold" << YAML::Value << tracking_options.klt_threshold;
    emitter << YAML::Key << "klt_min_tracked_ratio" << YAML::Value << tracking_options.klt_min_tracked_ratio;
    emitter << YAML::Key << "landmark_match_distance" << YAML::Value << tracking_options.landmark_match_distance;
    emitter << YAML::Key << "landmark_match_radius" << YAML::Value << tracking_options.landmark_match_radius;
    emitter << YAML::Key << "use_keylines" << YAML::Value << tracking_options.use_keylines;
    emitter << YAML::Key << "filter_epipolar" << YAML::Value << tracking_options.filter_epipolar;
    emitter << YAML::Key << "epipolar_threshold" << YAML::Value << tracking_options.epipolar_threshold;
    emitter << YAML::EndMap;
    emitter << YAML::Newline;

    

    return emitter;
}