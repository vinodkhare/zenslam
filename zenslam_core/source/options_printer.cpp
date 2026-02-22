#include "zenslam/options_printer.h"

#include <magic_enum/magic_enum.hpp>
#include <spdlog/spdlog.h>

namespace zenslam
{
    void options_printer::print(const all_options& opts) {
        SPDLOG_INFO("===== Configuration =====");
        SPDLOG_INFO("file: {}", opts.file.string());
        SPDLOG_INFO("log_level: {}", magic_enum::enum_name(opts.log_level));
        SPDLOG_INFO("log_pattern: {}", opts.log_pattern);
        SPDLOG_INFO("verb: {}", magic_enum::enum_name(opts.verb_));

        SPDLOG_INFO("===== Folder Options =====");
        print_folder(opts.folder);

        SPDLOG_INFO("===== GUI Options =====");
        print_gui(opts.gui);

        SPDLOG_INFO("===== SLAM Options =====");
        print_slam(opts.slam);
    }

    void options_printer::print_detection(const detection_options& opts) {
        SPDLOG_INFO("[detection] clahe_enabled: {}", opts.clahe_enabled);
        SPDLOG_INFO("[detection] stereo_rectify: {}", opts.stereo_rectify);
        SPDLOG_INFO("[detection] cell_size: [{}, {}]", opts.cell_size.width, opts.cell_size.height);
        SPDLOG_INFO("[detection] fast_threshold: {}", opts.fast_threshold);
        SPDLOG_INFO("[detection] keyline_max_length: {}", opts.keyline_max_length);
        SPDLOG_INFO("[detection] feature_detector: {}", magic_enum::enum_name(opts.feature_detector));
        SPDLOG_INFO("[detection] descriptor: {}", magic_enum::enum_name(opts.descriptor));
        SPDLOG_INFO("[detection] algorithm: {}", magic_enum::enum_name(opts.algorithm));
    }

    void options_printer::print_tracking(const tracking_options& opts) {
        SPDLOG_INFO("[tracking] klt_window_size: [{}, {}]", opts.klt_window_size.width, opts.klt_window_size.height);
        SPDLOG_INFO("[tracking] klt_max_level: {}", opts.klt_max_level);
        SPDLOG_INFO("[tracking] klt_threshold: {}", opts.klt_threshold);
        SPDLOG_INFO("[tracking] use_keylines: {}", opts.use_keylines);
    }

    void options_printer::print_triangulation(const triangulation_options& opts) {
        SPDLOG_INFO("[triangulation] min_disparity: {}", opts.min_disparity);
        SPDLOG_INFO("[triangulation] keyline_mask_margin: {}", opts.keyline_mask_margin);
        SPDLOG_INFO("[triangulation] min_angle: {}", opts.min_angle);
        SPDLOG_INFO("[triangulation] reprojection_threshold: {}", opts.reprojection_threshold);
        SPDLOG_INFO("[triangulation] min_depth: {}", opts.min_depth);
        SPDLOG_INFO("[triangulation] max_depth: {}", opts.max_depth);
    }

    void options_printer::print_keyframe(const keyframe_options& opts) {
        SPDLOG_INFO("[keyframe] min_frames: {}", opts.min_frames);
        SPDLOG_INFO("[keyframe] max_frames: {}", opts.max_frames);
        SPDLOG_INFO("[keyframe] min_translation: {}", opts.min_translation);
        SPDLOG_INFO("[keyframe] min_rotation_deg: {}", opts.min_rotation_deg);
        SPDLOG_INFO("[keyframe] min_tracked_ratio: {}", opts.min_tracked_ratio);
        SPDLOG_INFO("[keyframe] min_inliers: {}", opts.min_inliers);
    }

    void options_printer::print_lba(const lba_options& opts) {
        SPDLOG_INFO("[lba] max_iterations: {}", opts.max_iterations);
        SPDLOG_INFO("[lba] huber_delta: {}", opts.huber_delta);
        SPDLOG_INFO("[lba] refine_landmarks: {}", opts.refine_landmarks);
    }

    void options_printer::print_pnp(const pnp_options& opts) {
        SPDLOG_INFO("[pnp] iterations: {}", opts.iterations);
        SPDLOG_INFO("[pnp] threshold: {}", opts.threshold);
        SPDLOG_INFO("[pnp] confidence: {}", opts.confidence);
        SPDLOG_INFO("[pnp] use_refinement: {}", opts.use_refinement);
        SPDLOG_INFO("[pnp] min_refinement_inliers: {}", opts.min_refinement_inliers);
    }

    void options_printer::print_essential(const essential_options& opts) {
        SPDLOG_INFO("[essential] confidence: {}", opts.confidence);
        SPDLOG_INFO("[essential] threshold: {}", opts.threshold);
        SPDLOG_INFO("[essential] min_inliers: {}", opts.min_inliers);
    }

    void options_printer::print_rigid(const rigid_options& opts) {
        SPDLOG_INFO("[rigid] threshold: {}", opts.threshold);
        SPDLOG_INFO("[rigid] iterations: {}", opts.iterations);
        SPDLOG_INFO("[rigid] min_correspondences: {}", opts.min_correspondences);
    }

    void options_printer::print_slam(const slam_options& opts) {
        SPDLOG_INFO("[slam] matcher: {}", magic_enum::enum_name(opts.matcher));
        SPDLOG_INFO("[slam] matcher_ratio: {}", opts.matcher_ratio);
        SPDLOG_INFO("[slam] integrator_method: {}", magic_enum::enum_name(opts.integrator_method));
        SPDLOG_INFO("[slam] threshold_3d3d: {}", opts.threshold_3d3d);
        SPDLOG_INFO("[slam] threshold_3d2d: {}", opts.threshold_3d2d);
        SPDLOG_INFO("[slam] reprojection_threshold_3d2d: {}", opts.reprojection_threshold_3d2d);
        SPDLOG_INFO("[slam] epipolar_threshold: {}", opts.epipolar_threshold);
        SPDLOG_INFO("[slam] enable_frustum_culling: {}", opts.enable_frustum_culling);
        SPDLOG_INFO("[slam] frustum_margin: {}", opts.frustum_margin);

        print_detection(opts.detection);
        print_tracking(opts.tracking);
        print_triangulation(opts.triangulation);
        print_keyframe(opts.keyframe);
        print_lba(opts.lba);
        print_pnp(opts.pnp);
        print_essential(opts.essential);
        print_rigid(opts.rigid);
    }

    void options_printer::print_gui(const gui_options& opts) {
        SPDLOG_INFO("[gui] show_keypoints: {}", opts.show_keypoints);
        SPDLOG_INFO("[gui] show_keylines: {}", opts.show_keylines);
        SPDLOG_INFO("[gui] keyline_thickness: {}", opts.keyline_thickness);
        SPDLOG_INFO("[gui] point_cloud_opacity: {}", opts.point_cloud_opacity);
        SPDLOG_INFO("[gui] point_size: {}", opts.point_size);
    }

    void options_printer::print_folder(const folder_options& opts) {
        SPDLOG_INFO("[folder] root: {}", opts.root.string());
        SPDLOG_INFO("[folder] left: {}", opts.left.string());
        SPDLOG_INFO("[folder] right: {}", opts.right.string());
        SPDLOG_INFO("[folder] output: {}", opts.output.string());
        SPDLOG_INFO("[folder] calibration_file: {}", opts.calibration_file.string());
        SPDLOG_INFO("[folder] groundtruth_file: {}", opts.groundtruth_file.string());
        SPDLOG_INFO("[folder] imu_calibration_file: {}", opts.imu_calibration_file.string());
        SPDLOG_INFO("[folder] imu_file: {}", opts.imu_file.string());
        SPDLOG_INFO("[folder] timescale: {}", opts.timescale);
    }

}  // namespace zenslam
