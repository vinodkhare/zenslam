#pragma once

#include <string>

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>

#include <opencv2/core/types.hpp>

#include "zenslam/detection_types.h"
#include "zenslam/integrator.h"
#include "zenslam/option.h"
#include "zenslam/options_base.h"

namespace YAML
{
    class Node;
}


namespace zenslam
{
    enum class matcher_type
    {
        BRUTE,
        KNN,
        FLANN
    };

    /// PnP RANSAC estimation configuration
    class pnp_options : public options_base<pnp_options, "pnp options", "pnp.">
    {
    public:
        ZENSLAM_DEFINE_OPTIONS(
            ((int, iterations, 1000, "PnP RANSAC maximum iterations"))
            ((float, threshold, 3.0f, "PnP RANSAC inlier threshold in pixels"))
            ((double, confidence, 0.99, "PnP RANSAC confidence level (0.0-1.0)"))
            ((bool, use_refinement, true, "Use iterative LM refinement after RANSAC pose estimation"))
            ((int, min_refinement_inliers, 4, "Minimum inliers required for PnP refinement"))
        )
    };

    /// Essential matrix estimation configuration
    class essential_options : public options_base<essential_options, "essential options", "essential.">
    {
    public:
        ZENSLAM_DEFINE_OPTIONS(
            ((double, confidence, 0.999, "Essential matrix RANSAC confidence level (0.0-1.0)"))
            ((double, threshold, 1.0, "Essential matrix RANSAC threshold in pixels"))
            ((int, min_inliers, 5, "Essential matrix minimum inliers"))
        )
    };

    /// 3D-3D rigid transformation configuration
    class rigid_options : public options_base<rigid_options, "rigid options", "rigid.">
    {
    public:
        ZENSLAM_DEFINE_OPTIONS(
            ((double, threshold, 0.1, "3D-3D rigid transformation RANSAC threshold in meters"))
            ((int, iterations, 1000, "3D-3D rigid transformation RANSAC maximum iterations"))
            ((int, min_correspondences, 3, "3D-3D rigid transformation minimum correspondences"))
        )
    };

    /// Keyframe selection configuration
    class keyframe_options : public options_base<keyframe_options, "keyframe options", "keyframe.">
    {
    public:
        ZENSLAM_DEFINE_OPTIONS(
            ((int, min_frames, 5, "Minimum frames between keyframes"))
            ((int, max_frames, 30, "Maximum frames between keyframes"))
            ((double, min_translation, 0.15, "Minimum translation (m) to trigger a keyframe"))
            ((double, min_rotation_deg, 10.0, "Minimum rotation (deg) to trigger a keyframe"))
            ((double, min_tracked_ratio, 0.6, "Minimum tracked ratio before forcing a keyframe"))
            ((int, min_inliers, 20, "Minimum inliers before forcing a keyframe"))
        )
    };

    /// Local bundle adjustment configuration
    class lba_options : public options_base<lba_options, "lba options", "lba.">
    {
    public:
        ZENSLAM_DEFINE_OPTIONS(
            ((int, window_size, 5, "Number of keyframes in the LBA window"))
            ((int, max_iterations, 10, "Maximum LBA solver iterations"))
            ((double, huber_threshold, 3.0, "Huber loss threshold in pixels"))
        )
    };

    /// SLAM algorithm configuration options
    class slam_options : public options_base<slam_options, "slam options", "slam.">
    {
    public:
        using options_description = boost::program_options::options_description;

        static auto description() -> options_description;

        // Override parse_yaml to handle nested structures
        static auto parse_yaml(const YAML::Node& node) -> slam_options;

        // Override print to handle nested structures
        void print() const;

        // Inherited from options_base:
        // static constexpr auto name() - returns "slam options"
        // static constexpr auto prefix() - returns "slam."
        // static auto parse_yaml(const YAML::Node& node) -> slam_options;
        // static void parse_cli(slam_options& options, const std::map<std::string, boost::program_options::basic_option<char>>& options_map, const boost::program_options::variables_map& vm);
        // void print() const;

        // Define all options using the auto-registration macro
        // Format: ((type, name, default_value, "description"))
        ZENSLAM_DEFINE_OPTIONS(
            ((bool, clahe_enabled, false, "Enable CLAHE (Contrast Limited Adaptive Histogram Equalization)"))
            ((bool, stereo_rectify, false, "Enable stereo rectification"))
            ((bool, use_parallel_detector, true, "Use parallel grid detector for feature detection"))
            ((cv::Size, cell_size, cv::Size(16, 16), "Grid cell size for feature detection"))
            ((feature_type, feature, feature_type::FAST, "Feature detector type"))
            ((descriptor_type, descriptor, descriptor_type::ORB, "Feature descriptor type"))
            ((matcher_type, matcher, matcher_type::BRUTE, "Feature matcher type"))
            ((integrator::method, integrator_method, integrator::method::ugpm, "IMU integration method"))
            ((double, matcher_ratio, 0.8, "Matcher ratio test threshold for kNN matching (0.0-1.0)"))
            ((int, fast_threshold, 10, "FAST feature detector threshold"))
            ((cv::Size, klt_window_size, cv::Size(31, 31), "KLT optical flow window size"))
            ((int, klt_max_level, 3, "KLT optical flow maximum pyramid level"))
            ((double, klt_threshold, 1.0, "KLT tracking threshold"))
            ((double, epipolar_threshold, 1.0, "Epipolar constraint threshold in pixels"))
            ((double, threshold_3d3d, 0.005, "3D-3D RANSAC pose estimation threshold in meters"))
            ((double, threshold_3d2d, 1.0, "3D-2D RANSAC pose estimation threshold in pixels"))
            ((bool, show_keypoints, true, "Show keypoints in visualization"))
            ((bool, show_keylines, true, "Show keylines in visualization"))
            ((cv::Scalar, keyline_single_color, cv::Scalar(0, 255, 0), "Keyline single color (BGR)"))
            ((cv::Scalar, keyline_match_color, cv::Scalar(0, 0, 255), "Keyline match color (BGR)"))
            ((int, keyline_thickness, 1, "Keyline line thickness in pixels"))
            ((int, keyline_mask_margin, 10, "Keyline mask margin in pixels"))
            ((double, triangulation_min_disparity, 2.0, "Keyline minimum average disparity across endpoints in pixels"))
            ((double, triangulation_min_angle, 15.0, "Keyline minimum triangulation angle at endpoints in degrees"))
            ((double, triangulation_reprojection_threshold, 1.0, "Keyline maximum average reprojection error across endpoints in pixels"))
            ((double, triangulation_min_depth, 1.0, "Minimum triangulation depth in meters"))
            ((double, triangulation_max_depth, 50.0, "Maximum triangulation depth in meters"))
            ((keyframe_options, keyframe, {}, "Keyframe selection configuration"))
            ((lba_options, lba, {}, "Local bundle adjustment configuration"))
            ((pnp_options, pnp, {}, "PnP RANSAC configuration"))
            ((essential_options, essential, {}, "Essential matrix configuration"))
            ((rigid_options, rigid, {}, "3D-3D rigid transformation configuration"))
        )

        void validate() const;
    };
} // namespace zenslam

