#include "zenslam/options_writer.h"

#include <fstream>

#include <magic_enum/magic_enum.hpp>

#include <spdlog/spdlog.h>

#include <yaml-cpp/emitter.h>

#include "zenslam/yaml_emitters.h"

auto zenslam::options_writer::write_detection(YAML::Emitter& emitter, const detection_options& detection_options) -> void
{
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "clahe_enabled" << YAML::Value << detection_options.clahe_enabled;
    emitter << YAML::Key << "stereo_rectify" << YAML::Value << detection_options.stereo_rectify;
    emitter << YAML::Key << "cell_size" << YAML::Value << YAML::Flow << YAML::BeginSeq << detection_options.cell_size.width << detection_options.cell_size.height << YAML::EndSeq;
    emitter << YAML::Key << "fast_threshold" << YAML::Value << detection_options.fast_threshold;
    emitter << YAML::Key << "keyline_max_length" << YAML::Value << detection_options.keyline_max_length;
    emitter << YAML::Key << "feature_detector" << YAML::Value << std::string { magic_enum::enum_name(detection_options.feature_detector) };
    emitter << YAML::Key << "descriptor" << YAML::Value << std::string { magic_enum::enum_name(detection_options.descriptor) };
    emitter << YAML::Key << "algorithm" << YAML::Value << std::string { magic_enum::enum_name(detection_options.algorithm) };
    emitter << YAML::EndMap;
}

auto zenslam::options_writer::write_tracking(YAML::Emitter& emitter, const tracking_options& tracking_options) -> void
{
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "klt_window_size" << YAML::Value << YAML::Flow << YAML::BeginSeq << tracking_options.klt_window_size.width << tracking_options.klt_window_size.height <<
        YAML::EndSeq;
    emitter << YAML::Key << "klt_max_level" << YAML::Value << tracking_options.klt_max_level;
    emitter << YAML::Key << "klt_threshold" << YAML::Value << tracking_options.klt_threshold;
    emitter << YAML::Key << "use_keylines" << YAML::Value << tracking_options.use_keylines;
    emitter << YAML::EndMap;
}

auto zenslam::options_writer::write_triangulation(YAML::Emitter& emitter, const triangulation_options& triangulation_options) -> void
{
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "min_disparity" << YAML::Value << triangulation_options.min_disparity;
    emitter << YAML::Key << "keyline_mask_margin" << YAML::Value << triangulation_options.keyline_mask_margin;
    emitter << YAML::Key << "min_angle" << YAML::Value << triangulation_options.min_angle;
    emitter << YAML::Key << "reprojection_threshold" << YAML::Value << triangulation_options.reprojection_threshold;
    emitter << YAML::Key << "min_depth" << YAML::Value << triangulation_options.min_depth;
    emitter << YAML::Key << "max_depth" << YAML::Value << triangulation_options.max_depth;
    emitter << YAML::EndMap;
}

auto zenslam::options_writer::write_keyframe(YAML::Emitter& emitter, const keyframe_options& keyframe_options) -> void
{
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "min_frames" << YAML::Value << keyframe_options.min_frames;
    emitter << YAML::Key << "max_frames" << YAML::Value << keyframe_options.max_frames;
    emitter << YAML::Key << "min_translation" << YAML::Value << keyframe_options.min_translation;
    emitter << YAML::EndMap;
}

auto zenslam::options_writer::write_lba(YAML::Emitter& emitter, const lba_options& lba_options) -> void
{
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "max_iterations" << YAML::Value << lba_options.max_iterations;
    emitter << YAML::Key << "huber_delta" << YAML::Value << lba_options.huber_delta;
    emitter << YAML::Key << "refine_landmarks" << YAML::Value << lba_options.refine_landmarks;
    emitter << YAML::EndMap;
}

auto zenslam::options_writer::write_pnp(YAML::Emitter& emitter, const pnp_options& pnp_options) -> void
{
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "iterations" << YAML::Value << pnp_options.iterations;
    emitter << YAML::Key << "threshold" << YAML::Value << pnp_options.threshold;
    emitter << YAML::Key << "confidence" << YAML::Value << pnp_options.confidence;
    emitter << YAML::Key << "use_refinement" << YAML::Value << pnp_options.use_refinement;
    emitter << YAML::Key << "min_refinement_inliers" << YAML::Value << pnp_options.min_refinement_inliers;
    emitter << YAML::EndMap;
}

auto zenslam::options_writer::write_essential(YAML::Emitter& emitter, const essential_options& essential) -> void
{
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "confidence" << YAML::Value << essential.confidence;
    emitter << YAML::Key << "threshold" << YAML::Value << essential.threshold;
    emitter << YAML::Key << "min_inliers" << YAML::Value << essential.min_inliers;
    emitter << YAML::EndMap;
}

auto zenslam::options_writer::write_rigid(YAML::Emitter& emitter, const rigid_options& rigid_options) -> void
{
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "threshold" << YAML::Value << rigid_options.threshold;
    emitter << YAML::Key << "iterations" << YAML::Value << rigid_options.iterations;
    emitter << YAML::Key << "min_correspondences" << YAML::Value << rigid_options.min_correspondences;
    emitter << YAML::EndMap;
}

auto zenslam::options_writer::write_slam(YAML::Emitter& emitter, const slam_options& slam_options) -> void
{
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "matcher" << YAML::Value << std::string { magic_enum::enum_name(slam_options.matcher) };
    emitter << YAML::Key << "matcher_ratio" << YAML::Value << YAML::Precision(3) << slam_options.matcher_ratio;
    emitter << YAML::Key << "integrator_method" << YAML::Value << std::string { magic_enum::enum_name(slam_options.integrator_method) };
    emitter << YAML::Key << "threshold_3d3d" << YAML::Value << slam_options.threshold_3d3d;
    emitter << YAML::Key << "threshold_3d2d" << YAML::Value << slam_options.threshold_3d2d;
    emitter << YAML::Key << "reprojection_threshold_3d2d" << YAML::Value << slam_options.reprojection_threshold_3d2d;
    emitter << YAML::Key << "epipolar_threshold" << YAML::Value << slam_options.epipolar_threshold;
    emitter << YAML::Key << "enable_frustum_culling" << YAML::Value << slam_options.enable_frustum_culling;
    emitter << YAML::Key << "frustum_margin" << YAML::Value << slam_options.frustum_margin;

    // Nested sections
    emitter << YAML::Key << "detection" << YAML::Value;
    write_detection(emitter, slam_options.detection);

    emitter << YAML::Key << "tracking" << YAML::Value;
    write_tracking(emitter, slam_options.tracking);

    emitter << YAML::Key << "triangulation" << YAML::Value;
    write_triangulation(emitter, slam_options.triangulation);

    emitter << YAML::Key << "keyframe" << YAML::Value;
    write_keyframe(emitter, slam_options.keyframe);

    emitter << YAML::Key << "lba" << YAML::Value;
    write_lba(emitter, slam_options.lba);

    emitter << YAML::Key << "pnp" << YAML::Value;
    write_pnp(emitter, slam_options.pnp);

    emitter << YAML::Key << "essential" << YAML::Value;
    write_essential(emitter, slam_options.essential);

    emitter << YAML::Key << "rigid" << YAML::Value;
    write_rigid(emitter, slam_options.rigid);

    emitter << YAML::EndMap;

    emitter << YAML::Newline;
}

void zenslam::options_writer::write(const all_options& opts)
{
    YAML::Emitter emitter;
    emitter.SetIndent(4);
    // emitter.SetMapFormat(YAML::Flow);
    // emitter.SetSeqFormat(YAML::Flow);

    emitter << YAML::BeginMap;

    emitter << YAML::Key << "file" << YAML::Value << opts.file.string();
    emitter << YAML::Key << "log_level" << YAML::Value << std::string { magic_enum::enum_name(opts.log_level) };
    emitter << YAML::Key << "log_pattern" << YAML::Value << opts.log_pattern;
    emitter << YAML::Key << "verb" << YAML::Value << std::string { magic_enum::enum_name(opts.verb_) };

    // Nested sections - use yaml_emitters overloads
    emitter << YAML::Key << "folder" << YAML::Value << opts.folder;
    emitter << YAML::Key << "gui" << YAML::Value << opts.gui;

    emitter << YAML::Key << "slam" << YAML::Value;
    write_slam(emitter, opts.slam);

    emitter << YAML::EndMap;

    // Output to file
    std::ofstream output_stream(opts.file);
    if (output_stream.is_open())
    {
        output_stream << emitter.c_str();
        output_stream.close();
        SPDLOG_INFO("Options written to {}", opts.file.string());
    }
    else
    {
        SPDLOG_ERROR("Failed to open file for writing: {}", opts.file.string());
    }
}
