#pragma once

#include <filesystem>

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>

#include <opencv2/core/types.hpp>

#include <spdlog/common.h>

#include "detection_types.h"
#include "verb.h"

namespace zenslam
{
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

            void print() const;
        } folder;

        class slam
        {
        public:
            bool            clahe_enabled         = { false };
            cv::Size        cell_size             = { 16, 16 };
            feature_type    feature               = { feature_type::FAST };
            descriptor_type descriptor            = { descriptor_type::ORB };
            int             fast_threshold        = { 10 };
            cv::Size        klt_window_size       = { 31, 31 };
            int             klt_max_level         = { 3 };
            double          klt_threshold         = { 1.0 };
            double          min_depth             = { 1.0 };  // in meters
            double          max_depth             = { 50.0 }; // in meters
            double          epipolar_threshold    = { 1.0 };
            double          triangulation_threshold = { 1.0 };   // in pixels
            double          threshold_3d3d        = { 0.005 }; // in meters - for 3D-3D RANSAC pose estimation
            double          threshold_3d2d        = { 1.0 };   // in pixels - for 3D-2D RANSAC pose estimation

            void print() const;
        } slam;

        void print() const;
    };
} // namespace zenslam