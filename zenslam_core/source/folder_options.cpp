#include "zenslam/folder_options.h"

#include <filesystem>

#include <boost/program_options.hpp>

#include <spdlog/spdlog.h>

#include <yaml-cpp/yaml.h>

#include "zenslam/option_parser.h"
#include "zenslam/option_parser_cli.h"
#include "zenslam/option_printer.h"

namespace zenslam
{
    auto folder_options::parse_yaml(const YAML::Node& node) -> folder_options
    {
        folder_options options;

        try
        {
            // Use auto-generated all_options() to parse all fields in a loop
            std::apply
            (
                [&node](auto&... opts)
                {
                    ((opts = option_parser::parse_yaml(opts, node)), ...);
                },

                options.all_options()
            );
        }
        catch (const YAML::Exception& e)
        {
            SPDLOG_ERROR("Error parsing folder options from YAML: {}", e.what());
        }

        return options;
    }

    void folder_options::parse_cli(
        folder_options&                                                          options,
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
                    // All folder options use "folder." prefix
                    auto parse_field = [&options_map, &vm](auto& opt)
                    {
                        opt = option_parser_cli::parse_cli(opt, "folder.", options_map, vm);
                    };

                    (parse_field(opts), ...);
                },

                options.all_options()
            );
        }
        catch (const std::exception& e)
        {
            SPDLOG_ERROR("Error parsing folder options from CLI: {}", e.what());
        }
    }

    boost::program_options::options_description folder_options::description()
    {
        const folder_options opts;

        boost::program_options::options_description description { "folder options" };

        // Helper to add an option to boost description with proper CLI flag name
        auto add_option = [&description](const auto& opt)
        {
            using T = std::decay_t<decltype(opt.value())>;

            // Build the CLI flag name with "folder-" prefix and dashes instead of underscores
            const std::string flag_name = "folder." + opt.name();

            if constexpr (std::is_same_v<T, std::filesystem::path>)
            {
                description.add_options()
                (
                    flag_name.c_str(),
                    boost::program_options::value<std::string>()->default_value(opt.value().string()),
                    opt.description().c_str()
                );
            }
            else if constexpr (std::is_same_v<T, double>)
            {
                description.add_options()
                (
                    flag_name.c_str(),
                    boost::program_options::value<double>()->default_value(opt.value()),
                    opt.description().c_str()
                );
            }
            else
            {
                // Generic case for other types
                description.add_options()
                (
                    flag_name.c_str(),
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

    void folder_options::validate() const
    {
        if (timescale <= 0.0)
            throw std::invalid_argument("folder.timescale must be > 0");

        // Non-fatal warnings for non-existent paths
        auto resolve = [this](const std::filesystem::path& p) -> std::filesystem::path { return p.is_absolute() ? p : root / p; };

        const auto root_path = root.value();
        if (!root_path.empty() && !std::filesystem::exists(root_path))
        {
            SPDLOG_WARN("folder.root does not exist: {}", root_path.string());
        }

        const auto left_path = resolve(left);
        if (!std::filesystem::exists(left_path))
        {
            SPDLOG_WARN("folder.left path does not exist: {}", left_path.string());
        }

        const auto right_path = resolve(right);
        if (!std::filesystem::exists(right_path))
        {
            SPDLOG_WARN("folder.right path does not exist: {}", right_path.string());
        }

        const auto calib_path = resolve(calibration_file);
        if (!std::filesystem::exists(calib_path))
        {
            SPDLOG_WARN("folder.calibration_file not found: {}", calib_path.string());
        }

        const auto gt_path = resolve(groundtruth_file);
        if (!std::filesystem::exists(gt_path))
        {
            SPDLOG_WARN("folder.groundtruth_file not found: {}", gt_path.string());
        }

        const auto imu_path = resolve(imu_calibration_file);
        if (!std::filesystem::exists(imu_path))
        {
            SPDLOG_WARN("folder.imu_calibration_file not found: {}", imu_path.string());
        }
    }

    void folder_options::print() const
    {
        // Use auto-generated all_options() to print all fields in a loop
        std::apply([](const auto&... opts)
        {
            (option_printer::print(opts), ...);
        }, all_options());
    }
} // namespace zenslam
