#pragma once

#include <filesystem>

#include <boost/program_options/options_description.hpp>

#include <opencv2/core/types.hpp>

#include <spdlog/common.h>

#include "detection_types.h"
#include "integrator.h"
#include "verb.h"

namespace zenslam
{
    enum class matcher_type
    {
        BRUTE,
        KNN,
        FLANN
    };

    class options
    {
    public:
        using options_description = boost::program_options::options_description;

        static options_description description();

        static options parse(int argc, char** argv);
        static options parse(const std::filesystem::path& path);

        std::filesystem::path     file        = { "options.yaml" };
        spdlog::level::level_enum log_level   = { spdlog::level::trace };
        std::string               log_pattern = { "[%Y-%m-%d %H:%M:%S.%e] [%n] [%^%l%$] %v" };
        verb                      verb        = { verb::RUN };

        class folder
        {
        public:
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
        } folder;

        class slam
        {
        public:
            // Returns the options description for SLAM-related CLI options
            static options_description description();

            bool            clahe_enabled         = { false };
            bool            stereo_rectify        = { false };
            bool            use_parallel_detector = { true }; // Use parallel grid detector
            cv::Size        cell_size             = { 16, 16 };
            feature_type    feature               = { feature_type::FAST };
            descriptor_type descriptor            = { descriptor_type::ORB };
            matcher_type    matcher               = { matcher_type::BRUTE };

            // IMU preintegration backend method (ugpm or lpm)
            integrator::method integrator_method = { integrator::method::ugpm };
            double             matcher_ratio     = { 0.8 };
            int                fast_threshold    = { 10 };
            cv::Size           klt_window_size   = { 31, 31 };
            int                klt_max_level     = { 3 };
            double             klt_threshold     = { 1.0 };

            double epipolar_threshold = { 1.0 };
            double threshold_3d3d     = { 0.005 }; // in meters - for 3D-3D RANSAC pose estimation
            double threshold_3d2d     = { 1.0 };   // in pixels - for 3D-2D RANSAC pose estimation
            bool   show_keypoints     = { true };  // show keypoints in visualization
            bool   show_keylines      = { true };  // show keylines in visualization

            // Visualization options for keylines
            cv::Scalar keyline_single_color = { 0, 255, 0 }; // default green (BGR)
            cv::Scalar keyline_match_color  = { 0, 0, 255 }; // default red (BGR)
            int        keyline_thickness    = { 1 };         // line thickness in pixels

            // Keyline triangulation thresholds (configurable; previously hard-coded)
            // Minimum acceptable average disparity between matched endpoints (pixels)
            double triangulation_min_disparity = { 2.0 };
            // Minimum acceptable triangulation angle at endpoints (degrees)
            double triangulation_min_angle = { 15 };
            // Maximum allowable reprojection error after triangulation (pixels)
            double triangulation_reprojection_threshold = { 1.0 }; // in pixels
            // Valid depth range for triangulated keylines (meters)
            double triangulation_min_depth = { 1.0 }; // in meters
            // Maximum depth for triangulated keylines (meters)
            double triangulation_max_depth = { 50.0 }; // in meters

            void validate() const;
            void print() const;
        } slam;

        void validate() const;
        void print() const;
    };
} // namespace zenslam
