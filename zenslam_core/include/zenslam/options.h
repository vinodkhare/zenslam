#pragma once

#include <filesystem>

#include <boost/program_options/options_description.hpp>

#include <opencv2/core/types.hpp>

#include <spdlog/common.h>

#include "detection_types.h"
#include "folder_options.h"
#include "integrator.h"
#include "verb.h"
#include "zenslam/option.h"

namespace zenslam
{
    enum class matcher_type
    {
        BRUTE,
        KNN,
        FLANN
    };

    #define ZENSLAM_OPTION(type, name, default_val, description) option<type> name = { default_val, #name, description }

    class options
    {
    public:
        using options_description = boost::program_options::options_description;

        static options_description description();

        static options parse(int argc, char** argv);
        static options parse(const std::filesystem::path& path);

        ZENSLAM_OPTION(std::filesystem::path, file, "options.yaml", "Path to the options YAML file");
        ZENSLAM_OPTION(spdlog::level::level_enum, log_level, spdlog::level::trace, "Logging level");
        ZENSLAM_OPTION(std::string, log_pattern, "[%Y-%b-%d %T.%e] [%^%l%$] %v", "Logging pattern");
        ZENSLAM_OPTION(verb, verb, verb::RUN, "Verbosity level");

        folder_options folder;

        class slam
        {
        public:
            static options_description description();

            bool            clahe_enabled         = { false };
            bool            stereo_rectify        = { false };
            bool            use_parallel_detector = { true };
            cv::Size        cell_size             = { 16, 16 };
            feature_type    feature               = { feature_type::FAST };
            descriptor_type descriptor            = { descriptor_type::ORB };
            matcher_type    matcher               = { matcher_type::BRUTE };

            integrator::method integrator_method = { integrator::method::ugpm };
            double             matcher_ratio     = { 0.8 };
            int                fast_threshold    = { 10 };
            cv::Size           klt_window_size   = { 31, 31 };
            int                klt_max_level     = { 3 };
            double             klt_threshold     = { 1.0 };

            double epipolar_threshold = { 1.0 };
            double threshold_3d3d     = { 0.005 };
            double threshold_3d2d     = { 1.0 };
            bool   show_keypoints     = { true };
            bool   show_keylines      = { true };

            cv::Scalar keyline_single_color = { 0, 255, 0 };
            cv::Scalar keyline_match_color  = { 0, 0, 255 };
            int        keyline_thickness    = { 1 };
            int        keyline_mask_margin  = { 10 };

            double triangulation_min_disparity = { 2.0 };
            double triangulation_min_angle = { 15 };
            double triangulation_reprojection_threshold = { 1.0 };
            double triangulation_min_depth = { 1.0 };
            double triangulation_max_depth = { 50.0 };

            void validate() const;
            void print() const;
        } slam;

        void validate() const;
        void print() const;
    };

    #undef OPTION
} // namespace zenslam
