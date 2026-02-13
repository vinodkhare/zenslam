#include "zenslam/slam_options.h"

#include <algorithm>

#include <boost/program_options.hpp>

#include <magic_enum/magic_enum.hpp>

#include <yaml-cpp/yaml.h>

#include "zenslam/option_parser.h"
#include "zenslam/utils.h"

namespace zenslam
{
    // parse_yaml() and parse_cli() are now inherited from options_base

    boost::program_options::options_description slam_options::description()
    {
        const slam_options opts;

        boost::program_options::options_description description { "slam options" };

        // Helper to add an option to boost description with proper CLI flag name
        auto add_option = [&description](const auto& opt, const std::string& flag_prefix = "")
        {
            using T = std::decay_t<decltype(opt.value())>;

            if constexpr (std::is_same_v<T, bool>)
            {
                description.add_options()
                (
                    std::string("slam." + opt.name()).c_str(),
                    boost::program_options::bool_switch()->default_value(opt.value()),
                    opt.description().c_str()
                );
            }
            else if constexpr (std::is_same_v<T, cv::Size>)
            {
                // Skip cv::Size for now - complex to represent in CLI
                // Can be set via YAML
            }
            else if constexpr (std::is_same_v<T, cv::Scalar>)
            {
                // Skip cv::Scalar for now - complex to represent in CLI
                // Can be set via YAML
            }
            else if constexpr (std::is_enum_v<T>)
            {
                const std::string enum_desc = opt.description() + std::string(" - pick one of: ") + utils::to_string(magic_enum::enum_names<T>());

                description.add_options()
                (
                    std::string("slam." + opt.name()).c_str(),
                    boost::program_options::value<std::string>()->default_value(std::string(magic_enum::enum_name(opt.value()))),
                    enum_desc.c_str()
                );
            }
            else
            {
                // Generic case for other types
                description.add_options()
                (
                    std::string("slam." + opt.name()).c_str(),
                    boost::program_options::value<T>()->default_value(opt.value()),
                    opt.description().c_str()
                );
            }
        };

        // Use all_options() to automatically add all fields
        std::apply([&add_option](const auto&... options)
        {
            (add_option(options), ...);
        }, opts.all_options());

        return description;
    }

    void slam_options::validate() const
    {
        if (keyline_thickness < 1)
            throw std::invalid_argument("slam.keyline_thickness must be >= 1");
        if (epipolar_threshold < 0.0)
            throw std::invalid_argument("slam.epipolar_threshold must be >= 0");
        if (matcher_ratio <= 0.0 || matcher_ratio >= 1.0)
            throw std::invalid_argument("slam.matcher_ratio must be in (0, 1)");
        if (triangulation_min_disparity < 0.0)
            throw std::invalid_argument("slam.keyline_min_disparity_px must be >= 0");
        if (triangulation_min_angle < 0.0)
            throw std::invalid_argument("slam.keyline_min_triangulation_angle_deg must be >= 0");
        if (triangulation_reprojection_threshold <= 0.0)
            throw std::invalid_argument("slam.triangulation_reprojection_threshold must be > 0");
        if (triangulation_min_depth <= 0.0 || triangulation_max_depth <= 0.0 || triangulation_min_depth >= triangulation_max_depth)
            throw std::invalid_argument("slam depth range invalid: ensure 0 < min_depth < max_depth");
        if (klt_window_size.value().width <= 0 || klt_window_size.value().height <= 0)
            throw std::invalid_argument("slam.klt_window_size must be positive");
        if (klt_max_level < 0)
            throw std::invalid_argument("slam.klt_max_level must be >= 0");
    }

    // print() is now inherited from options_base
} // namespace zenslam

