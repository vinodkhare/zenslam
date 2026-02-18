#include "zenslam/slam_options.h"

#include <algorithm>

#include <boost/program_options.hpp>

#include <magic_enum/magic_enum.hpp>

#include <spdlog/spdlog.h>

#include <yaml-cpp/yaml.h>

#include "zenslam/option_parser.h"
#include "zenslam/option_printer.h"
#include "zenslam/utils/utils.h"

namespace zenslam
{
    // Override parse_yaml to handle nested structures
    slam_options slam_options::parse_yaml(const YAML::Node& node)
    {
        slam_options options;

        try
        {
            // Parse all options including nested ones
            std::apply
            (
                [&node](auto&... opts)
                {
                    // Parse each option, handling nested options classes specially
                    auto parse_opt = [&node](auto& opt)
                    {
                        using T = std::decay_t<decltype(opt.value())>;
                        
                        // Check if this is a nested options class
                        if constexpr (std::is_base_of_v<pnp_options, T> || 
                                      std::is_base_of_v<essential_options, T> ||
                                      std::is_base_of_v<rigid_options, T> ||
                                      std::is_base_of_v<keyframe_options, T> ||
                                      std::is_base_of_v<lba_options, T> ||
                                      std::is_base_of_v<gui_options, T>)
                        {
                            // Parse nested options class using its own parse_yaml
                            if (const auto nested_node = node[opt.name()])
                            {
                                opt = T::parse_yaml(nested_node);
                            }
                        }
                        else
                        {
                            // Parse regular option
                            opt = option_parser::parse_yaml(opt, node);
                        }
                    };
                    
                    (parse_opt(opts), ...);
                },
                options.all_options()
            );
        }
        catch (const YAML::Exception& e)
        {
            SPDLOG_ERROR("Error parsing {} from YAML: {}", std::string(slam_options::name()), e.what());
        }

        return options;
    }

    void slam_options::print() const
    {
        std::apply([](const auto&... opts)
        {
            auto print_opt = [](const auto& opt)
            {
                using T = std::decay_t<decltype(opt.value())>;
                
                // Check if this is a nested options class
                if constexpr (std::is_base_of_v<pnp_options, T> || 
                              std::is_base_of_v<essential_options, T> ||
                              std::is_base_of_v<rigid_options, T> ||
                              std::is_base_of_v<keyframe_options, T> ||
                              std::is_base_of_v<lba_options, T> ||
                              std::is_base_of_v<gui_options, T>)
                {
                    // Print nested options using their own print() method
                    SPDLOG_INFO("{}:", opt.name());
                    opt.value().print();
                }
                else
                {
                    // Print regular option
                    option_printer::print(opt);
                }
            };
            
            (print_opt(opts), ...);
        }, all_options());
    }

    boost::program_options::options_description slam_options::description()
    {
        const slam_options opts;

        boost::program_options::options_description description { "slam options" };

        // Helper to add an option to boost description with proper CLI flag name
        auto add_option = [&description](const auto& opt, const std::string& flag_prefix = "slam.")
        {
            using T = std::decay_t<decltype(opt.value())>;

            // Check if this is a nested options class
            if constexpr (std::is_base_of_v<pnp_options, T> || 
                          std::is_base_of_v<essential_options, T> ||
                          std::is_base_of_v<rigid_options, T> ||
                          std::is_base_of_v<keyframe_options, T> ||
                          std::is_base_of_v<lba_options, T> ||
                          std::is_base_of_v<gui_options, T>)
            {
                // Recursively add nested options
                std::apply([&description, &flag_prefix, &opt](const auto&... nested_opts)
                {
                    auto add_nested = [&description, &flag_prefix, &opt](const auto& nested_opt)
                    {
                        using NestedT = std::decay_t<decltype(nested_opt.value())>;
                        const std::string nested_prefix = flag_prefix + opt.name() + ".";
                        
                        if constexpr (std::is_same_v<NestedT, bool>)
                        {
                            description.add_options()
                            (
                                std::string(nested_prefix + nested_opt.name()).c_str(),
                                boost::program_options::bool_switch()->default_value(nested_opt.value()),
                                nested_opt.description().c_str()
                            );
                        }
                        else if constexpr (std::is_same_v<NestedT, cv::Size>)
                        {
                            // Skip cv::Size for now - complex to represent in CLI
                            // Can be set via YAML
                        }
                        else if constexpr (std::is_same_v<NestedT, cv::Scalar>)
                        {
                            // Skip cv::Scalar for now - complex to represent in CLI
                            // Can be set via YAML
                        }
                        else
                        {
                            description.add_options()
                            (
                                std::string(nested_prefix + nested_opt.name()).c_str(),
                                boost::program_options::value<NestedT>()->default_value(nested_opt.value()),
                                nested_opt.description().c_str()
                            );
                        }
                    };
                    
                    (add_nested(nested_opts), ...);
                }, opt.value().all_options());
            }
            else if constexpr (std::is_same_v<T, bool>)
            {
                description.add_options()
                (
                    std::string(flag_prefix + opt.name()).c_str(),
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
                    std::string(flag_prefix + opt.name()).c_str(),
                    boost::program_options::value<std::string>()->default_value(std::string(magic_enum::enum_name(opt.value()))),
                    enum_desc.c_str()
                );
            }
            else
            {
                // Generic case for other types
                description.add_options()
                (
                    std::string(flag_prefix + opt.name()).c_str(),
                    boost::program_options::value<T>()->default_value(opt.value()),
                    opt.description().c_str()
                );
            }
        };

        // Use all_options() to automatically add all fields (including nested ones)
        std::apply([&add_option](const auto&... options)
        {
            (add_option(options), ...);
        }, opts.all_options());

        return description;
    }

    void slam_options::validate() const
    {
        if (gui->keyline_thickness < 1)
            throw std::invalid_argument("slam.gui.keyline_thickness must be >= 1");
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
        if (keyframe.value().min_frames < 1)
            throw std::invalid_argument("slam.keyframe.min_frames must be >= 1");
        if (keyframe.value().max_frames < keyframe.value().min_frames)
            throw std::invalid_argument("slam.keyframe.max_frames must be >= min_frames");
        if (keyframe.value().min_translation < 0.0)
            throw std::invalid_argument("slam.keyframe.min_translation must be >= 0");
        if (keyframe.value().min_rotation_deg < 0.0)
            throw std::invalid_argument("slam.keyframe.min_rotation_deg must be >= 0");
        if (keyframe.value().min_tracked_ratio < 0.0 || keyframe.value().min_tracked_ratio > 1.0)
            throw std::invalid_argument("slam.keyframe.min_tracked_ratio must be in [0, 1]");
        if (keyframe.value().min_inliers < 0)
            throw std::invalid_argument("slam.keyframe.min_inliers must be >= 0");
        if (lba.value().max_iterations < 1)
            throw std::invalid_argument("slam.lba.max_iterations must be >= 1");
        if (lba.value().huber_delta <= 0.0)
            throw std::invalid_argument("slam.lba.huber_delta must be > 0");
    }

    // print() is now inherited from options_base
} // namespace zenslam

