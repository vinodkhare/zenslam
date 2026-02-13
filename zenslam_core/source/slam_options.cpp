#include "zenslam/slam_options.h"

#include <algorithm>

#include <boost/program_options.hpp>

#include <magic_enum/magic_enum.hpp>

#include <spdlog/spdlog.h>

#include <yaml-cpp/yaml.h>

#include "zenslam/option_parser.h"
#include "zenslam/option_parser_cli.h"
#include "zenslam/option_printer.h"
#include "zenslam/utils.h"

namespace zenslam
{
    auto slam_options::parse_yaml(const YAML::Node& node) -> slam_options
    {
        slam_options options;

        try
        {
            // Use auto-generated all_options() to parse all fields in a loop
            std::apply([&node](auto&... opts)
            {
                ((opts = option_parser::parse_yaml(opts, node)), ...);
            }, options.all_options());
        }
        catch (const YAML::Exception& e)
        {
            SPDLOG_ERROR("Error parsing slam options from YAML: {}", e.what());
        }

        return options;
    }

    void slam_options::parse_cli(
        slam_options&                                                            options,
        const std::map<std::string, boost::program_options::basic_option<char>>& options_map,
        const boost::program_options::variables_map&                             vm)
    {
        try
        {
            // Use auto-generated all_options() to parse all fields in a loop
            std::apply
            (
                [&options_map, &vm](auto&... opts)
                {
                    // All slam options use "slam." prefix
                    auto parse_field = [&options_map, &vm](auto& opt)
                    {
                        opt = option_parser_cli::parse_cli(opt, "slam.", options_map, vm);
                    };

                    (parse_field(opts), ...);
                },

                options.all_options()
            );
        }
        catch (const std::exception& e)
        {
            SPDLOG_ERROR("Error parsing slam options from CLI: {}", e.what());
        }
    }

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

    void slam_options::print() const
    {
        // Use auto-generated all_options() to print all fields in a loop
        std::apply([](const auto&... opts)
        {
            (option_printer::print(opts), ...);
        }, all_options());
    }
} // namespace zenslam

