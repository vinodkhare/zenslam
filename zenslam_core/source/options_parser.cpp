#include "zenslam/options_parser.h"

#include <spdlog/spdlog.h>
#include <magic_enum/magic_enum.hpp>

namespace zenslam
{
    // ========================================================================
    // Main Entry Points
    // ========================================================================

    all_options options_parser::load(const std::filesystem::path& yaml_file) {
        try {
            auto node = YAML::LoadFile(yaml_file.string());
            return load_from_node(node);
        } catch (const std::exception& e) {
            SPDLOG_ERROR("Failed to load options from {}: {}", yaml_file.string(), e.what());
            return all_options {};
        }
    }

    all_options options_parser::load_from_node(const YAML::Node& root) {
        all_options opts;
        opts.file = get_or_default(root, "file", opts.file);
        opts.log_pattern = get_or_default(root, "log_pattern", opts.log_pattern);

        // Parse log level
        if (root["log_level"]) {
            auto level_str = root["log_level"].as<std::string>();
            if (auto level = magic_enum::enum_cast<spdlog::level::level_enum>(level_str)) {
                opts.log_level = level.value();
            }
        }

        // Parse verb enum
        if (root["verb"]) {
            auto verb_str = root["verb"].as<std::string>();
            if (auto v = magic_enum::enum_cast<zenslam::verb>(verb_str)) {
                opts.verb_ = v.value();
            }
        }

        // Parse nested sections
        opts.folder = parse_folder(root["folder"]);
        opts.gui = parse_gui(root["gui"]);
        opts.slam = parse_slam(root["slam"]);

        return opts;
    }

    // ========================================================================
    // Detection Options
    // ========================================================================

    detection_options options_parser::parse_detection(const YAML::Node& node) {
        detection_options opts;
        if (!node || !node.IsMap()) return opts;

        opts.clahe_enabled = get_or_default(node, "clahe_enabled", opts.clahe_enabled);
        opts.stereo_rectify = get_or_default(node, "stereo_rectify", opts.stereo_rectify);
        opts.use_parallel_detector = get_or_default(node, "use_parallel_detector", opts.use_parallel_detector);
        opts.cell_size = get_size(node, "cell_size", opts.cell_size);
        opts.fast_threshold = get_or_default(node, "fast_threshold", opts.fast_threshold);

        return opts;
    }

    // ========================================================================
    // Tracking Options
    // ========================================================================

    tracking_options options_parser::parse_tracking(const YAML::Node& node) {
        tracking_options opts;
        if (!node || !node.IsMap()) return opts;

        opts.klt_window_size = get_size(node, "klt_window_size", opts.klt_window_size);
        opts.klt_max_level = get_or_default(node, "klt_max_level", opts.klt_max_level);
        opts.klt_threshold = get_or_default(node, "klt_threshold", opts.klt_threshold);
        opts.klt_min_tracked_ratio = get_or_default(node, "klt_min_tracked_ratio", opts.klt_min_tracked_ratio);
        opts.landmark_match_distance = get_or_default(node, "landmark_match_distance", opts.landmark_match_distance);
        opts.landmark_match_radius = get_or_default(node, "landmark_match_radius", opts.landmark_match_radius);

        return opts;
    }

    // ========================================================================
    // Triangulation Options
    // ========================================================================

    triangulation_options options_parser::parse_triangulation(const YAML::Node& node) {
        triangulation_options opts;
        if (!node || !node.IsMap()) return opts;

        opts.min_disparity = get_or_default(node, "min_disparity", opts.min_disparity);
        opts.keyline_mask_margin = get_or_default(node, "keyline_mask_margin", opts.keyline_mask_margin);
        opts.min_angle = get_or_default(node, "min_angle", opts.min_angle);
        opts.reprojection_threshold = get_or_default(node, "reprojection_threshold", opts.reprojection_threshold);
        opts.min_depth = get_or_default(node, "min_depth", opts.min_depth);
        opts.max_depth = get_or_default(node, "max_depth", opts.max_depth);

        return opts;
    }

    // ========================================================================
    // Keyframe Options
    // ========================================================================

    keyframe_options options_parser::parse_keyframe(const YAML::Node& node) {
        keyframe_options opts;
        if (!node || !node.IsMap()) return opts;

        opts.min_frames = get_or_default(node, "min_frames", opts.min_frames);
        opts.max_frames = get_or_default(node, "max_frames", opts.max_frames);
        opts.min_translation = get_or_default(node, "min_translation", opts.min_translation);
        opts.min_rotation_deg = get_or_default(node, "min_rotation_deg", opts.min_rotation_deg);
        opts.min_tracked_ratio = get_or_default(node, "min_tracked_ratio", opts.min_tracked_ratio);
        opts.min_inliers = get_or_default(node, "min_inliers", opts.min_inliers);

        return opts;
    }

    // ========================================================================
    // LBA Options
    // ========================================================================

    lba_options options_parser::parse_lba(const YAML::Node& node) {
        lba_options opts;
        if (!node || !node.IsMap()) return opts;

        opts.max_iterations = get_or_default(node, "max_iterations", opts.max_iterations);
        opts.huber_delta = get_or_default(node, "huber_delta", opts.huber_delta);
        opts.refine_landmarks = get_or_default(node, "refine_landmarks", opts.refine_landmarks);

        return opts;
    }

    // ========================================================================
    // PnP Options
    // ========================================================================

    pnp_options options_parser::parse_pnp(const YAML::Node& node) {
        pnp_options opts;
        if (!node || !node.IsMap()) return opts;

        opts.iterations = get_or_default(node, "iterations", opts.iterations);
        opts.threshold = get_or_default(node, "threshold", opts.threshold);
        opts.confidence = get_or_default(node, "confidence", opts.confidence);
        opts.use_refinement = get_or_default(node, "use_refinement", opts.use_refinement);
        opts.min_refinement_inliers = get_or_default(node, "min_refinement_inliers", opts.min_refinement_inliers);

        return opts;
    }

    // ========================================================================
    // Essential Options
    // ========================================================================

    essential_options options_parser::parse_essential(const YAML::Node& node) {
        essential_options opts;
        if (!node || !node.IsMap()) return opts;

        opts.confidence = get_or_default(node, "confidence", opts.confidence);
        opts.threshold = get_or_default(node, "threshold", opts.threshold);
        opts.min_inliers = get_or_default(node, "min_inliers", opts.min_inliers);

        return opts;
    }

    // ========================================================================
    // Rigid Options
    // ========================================================================

    rigid_options options_parser::parse_rigid(const YAML::Node& node) {
        rigid_options opts;
        if (!node || !node.IsMap()) return opts;

        opts.threshold = get_or_default(node, "threshold", opts.threshold);
        opts.iterations = get_or_default(node, "iterations", opts.iterations);
        opts.min_correspondences = get_or_default(node, "min_correspondences", opts.min_correspondences);

        return opts;
    }

    // ========================================================================
    // SLAM Options (aggregates all sub-modules)
    // ========================================================================

    slam_options options_parser::parse_slam(const YAML::Node& node) {
        slam_options opts;
        if (!node || !node.IsMap()) return opts;

        // Parse feature detector/descriptor/matcher
        if (node["feature"]) {
            auto feature_str = node["feature"].as<std::string>();
            if (auto f = magic_enum::enum_cast<feature_type>(feature_str)) {
                opts.feature_detector = f.value();
            }
        }

        if (node["descriptor"]) {
            auto desc_str = node["descriptor"].as<std::string>();
            if (auto d = magic_enum::enum_cast<descriptor_type>(desc_str)) {
                opts.descriptor = d.value();
            }
        }

        if (node["matcher"]) {
            auto matcher_str = node["matcher"].as<std::string>();
            if (auto m = magic_enum::enum_cast<matcher_type>(matcher_str)) {
                opts.matcher = m.value();
            }
        }

        if (node["integrator_method"]) {
            auto method_str = node["integrator_method"].as<std::string>();
            if (auto m = magic_enum::enum_cast<integrator::method>(method_str)) {
                opts.integrator_method = m.value();
            }
        }

        // Top-level options
        opts.matcher_ratio = get_or_default(node, "matcher_ratio", opts.matcher_ratio);
        opts.threshold_3d3d = get_or_default(node, "threshold_3d3d", opts.threshold_3d3d);
        opts.threshold_3d2d = get_or_default(node, "threshold_3d2d", opts.threshold_3d2d);
        opts.reprojection_threshold_3d2d = get_or_default(node, "reprojection_threshold_3d2d", opts.reprojection_threshold_3d2d);
        opts.epipolar_threshold = get_or_default(node, "epipolar_threshold", opts.epipolar_threshold);
        opts.enable_frustum_culling = get_or_default(node, "enable_frustum_culling", opts.enable_frustum_culling);
        opts.frustum_margin = get_or_default(node, "frustum_margin", opts.frustum_margin);

        // Nested module configurations
        opts.detection = parse_detection(node["detection"]);
        opts.tracking = parse_tracking(node["tracking"]);
        opts.triangulation = parse_triangulation(node["triangulation"]);
        opts.keyframe = parse_keyframe(node["keyframe"]);
        opts.lba = parse_lba(node["lba"]);
        opts.pnp = parse_pnp(node["pnp"]);
        opts.essential = parse_essential(node["essential"]);
        opts.rigid = parse_rigid(node["rigid"]);

        return opts;
    }

    // ========================================================================
    // GUI Options
    // ========================================================================

    gui_options options_parser::parse_gui(const YAML::Node& node) {
        gui_options opts;
        if (!node || !node.IsMap()) return opts;

        opts.show_keypoints = get_or_default(node, "show_keypoints", opts.show_keypoints);
        opts.show_keylines = get_or_default(node, "show_keylines", opts.show_keylines);
        opts.keyline_single_color = get_scalar(node, "keyline_single_color", opts.keyline_single_color);
        opts.keyline_match_color = get_scalar(node, "keyline_match_color", opts.keyline_match_color);
        opts.keyline_thickness = get_or_default(node, "keyline_thickness", opts.keyline_thickness);
        opts.point_cloud_opacity = get_or_default(node, "point_cloud_opacity", opts.point_cloud_opacity);
        opts.point_size = get_or_default(node, "point_size", opts.point_size);

        return opts;
    }

    // ========================================================================
    // Folder Options
    // ========================================================================

    folder_options options_parser::parse_folder(const YAML::Node& node) {
        folder_options opts;
        if (!node || !node.IsMap()) return opts;

        opts.root = get_or_default(node, "root", opts.root);
        opts.left = get_or_default(node, "left", opts.left);
        opts.right = get_or_default(node, "right", opts.right);
        opts.output = get_or_default(node, "output", opts.output);
        opts.calibration_file = get_or_default(node, "calibration_file", opts.calibration_file);
        opts.groundtruth_file = get_or_default(node, "groundtruth_file", opts.groundtruth_file);
        opts.imu_calibration_file = get_or_default(node, "imu_calibration_file", opts.imu_calibration_file);
        opts.imu_file = get_or_default(node, "imu_file", opts.imu_file);
        opts.timescale = get_or_default(node, "timescale", opts.timescale);

        return opts;
    }

    // ========================================================================
    // Helper Methods
    // ========================================================================

    cv::Size options_parser::get_size(const YAML::Node& node, const std::string& key, const cv::Size& default_val) {
        if (!node || !node[key]) return default_val;
        try {
            if (node[key].IsSequence() && node[key].size() >= 2) {
                return cv::Size(node[key][0].as<int>(), node[key][1].as<int>());
            }
        } catch (const std::exception&) {
        }
        return default_val;
    }

    cv::Scalar options_parser::get_scalar(const YAML::Node& node, const std::string& key, const cv::Scalar& default_val) {
        if (!node || !node[key]) return default_val;
        try {
            if (node[key].IsSequence() && node[key].size() >= 3) {
                return cv::Scalar(node[key][0].as<double>(),
                                 node[key][1].as<double>(),
                                 node[key][2].as<double>());
            }
        } catch (const std::exception&) {
        }
        return default_val;
    }

}  // namespace zenslam
