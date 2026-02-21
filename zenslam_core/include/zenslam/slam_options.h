#pragma once

#include <string>

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>

#include <opencv2/core/types.hpp>

#include "zenslam/detection/detection_types.h"
#include "zenslam/motion/integrator.h"
#include "zenslam/option.h"
#include "zenslam/options_base.h"
#include "zenslam/gui_options.h"
#include "zenslam/pnp_options.h"
#include "zenslam/essential_options.h"
#include "zenslam/rigid_options.h"
#include "zenslam/keyframe_options.h"
#include "zenslam/lba_options.h"

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

    /// SLAM algorithm configuration options
    class slam_options : public options_base<slam_options, "slam options", "slam">
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
            ((double, triangulation_min_disparity, 2.0, "Keyline minimum average disparity across endpoints in pixels"))
            ((int, keyline_mask_margin, 10, "Keyline mask margin in pixels"))
            ((double, triangulation_min_angle, 15.0, "Keyline minimum triangulation angle at endpoints in degrees"))
            ((double, triangulation_reprojection_threshold, 1.0, "Keyline maximum average reprojection error across endpoints in pixels"))
            ((double, triangulation_min_depth, 0.1, "Minimum triangulation depth in meters"))
            ((double, triangulation_max_depth, 100.0, "Maximum triangulation depth in meters"))
            ((double, reprojection_threshold_3d2d, 2.0, "3D-2D reprojection error threshold in pixels for landmark matching"))
            ((double, frustum_margin, 50.0, "Margin in pixels for frustum culling (negative = exclude, positive = include beyond image bounds)"))
            ((bool, enable_frustum_culling, true, "Enable frustum culling for 3D-2D landmark matching"))
            ((keyframe_options, keyframe, {}, "Keyframe selection configuration"))
            ((lba_options, lba, {}, "Local bundle adjustment configuration"))
            ((pnp_options, pnp, {}, "PnP RANSAC configuration"))
            ((essential_options, essential, {}, "Essential matrix configuration"))
            ((rigid_options, rigid, {}, "3D-3D rigid transformation configuration"))
        )

        void validate() const;
    };
} // namespace zenslam

