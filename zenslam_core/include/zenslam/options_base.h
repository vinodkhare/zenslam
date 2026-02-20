#pragma once

#include <filesystem>
#include <map>
#include <string>
#include <string_view>

#include <boost/program_options/option.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>

#include <opencv2/core/types.hpp>

#include <spdlog/spdlog.h>

#include <yaml-cpp/yaml.h>

#include "zenslam/option_parser.h"
#include "zenslam/option_parser_cli.h"
#include "zenslam/option_printer.h"

namespace zenslam
{
    /**
     * @brief Helper struct to hold compile-time string constants
     */
    template <size_t N>
    struct fixed_string
    {
        constexpr fixed_string(const char (&str)[N])
        {
            std::copy_n(str, N, value);
        }

        constexpr operator std::string_view() const
        {
            return std::string_view(value, N - 1);
        }

        char value[N];
    };

    /**
     * @brief CRTP base class providing common functionality for options classes
     * @tparam Derived The derived options class (e.g., folder_options, slam_options)
     * @tparam Name The name/description of this options group
     * @tparam Prefix The CLI prefix for these options (e.g., "folder", "slam") - dot is added automatically
     *
     * This base class abstracts away the common implementation pattern for:
     * - Parsing from YAML
     * - Parsing from CLI
     * - Generating boost::program_options descriptions
     * - Printing options
     *
     * Derived classes must:
     * 1. Use ZENSLAM_DEFINE_OPTIONS macro to define their options
     * 2. Implement validate() for custom validation logic
     * 3. Pass Name and Prefix as template parameters to options_base
     */
    template <typename Derived, fixed_string Name, fixed_string Prefix>
    class options_base
    {
    public:
        using options_description = boost::program_options::options_description;

        /**
         * @brief Get the name/description of this options group
         * @return The options group name
         */
        static constexpr std::string_view name()
        {
            return Name;
        }

        /**
         * @brief Get the CLI prefix for these options
         * @return The CLI prefix string
         */
        static constexpr std::string_view prefix()
        {
            return Prefix;
        }

        /**
         * @brief Parse options from a YAML node
         * @param node The YAML node containing the options
         * @return A new instance of Derived with parsed values
         */
        static auto parse_yaml(const YAML::Node& node) -> Derived
        {
            Derived options;

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
                SPDLOG_ERROR("Error parsing {} from YAML: {}", std::string(Derived::name()), e.what());
            }

            return options;
        }

        /**
         * @brief Parse options from CLI arguments
         * @param options The options instance to update
         * @param options_map Map of CLI options
         * @param vm Variables map from boost::program_options
         */
        static void parse_cli(
            Derived&                                                                 options,
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
                        auto parse_field = [&options_map, &vm](auto& opt)
                        {
                            opt = option_parser_cli::parse_cli(opt, std::string(Derived::prefix()) + ".", options_map, vm);
                        };

                        (parse_field(opts), ...);
                    },
                    options.all_options()
                );
            }
            catch (const std::exception& e)
            {
                SPDLOG_ERROR("Error parsing {} from CLI: {}", std::string(Derived::name()), e.what());
            }
        }

        /**
         * @brief Print all options to the log
         */
        void print() const
        {
            const auto& derived = static_cast<const Derived&>(*this);
            std::apply([](const auto&... opts)
            {
                (option_printer::print(opts), ...);
            }, derived.all_options());
        }

        /**
         * @brief Generate boost::program_options description for CLI
         * Default implementation handles common types. Derived classes can override for special handling.
         * @return options_description for boost::program_options
         */
        static auto description() -> options_description
        {
            const Derived opts;
            options_description desc { std::string(Derived::name()) };

            // Helper to add an option to boost description with proper CLI flag name
            auto add_option = [&desc](const auto& opt)
            {
                using T = std::decay_t<decltype(opt.value())>;
                const std::string flag_name = std::string(Derived::prefix()) + "." + opt.name();

                if constexpr (std::is_same_v<T, bool>)
                {
                    desc.add_options()
                    (
                        flag_name.c_str(),
                        boost::program_options::value<bool>()->default_value(opt.value()),
                        opt.description().c_str()
                    );
                }
                else if constexpr (std::is_same_v<T, int>)
                {
                    desc.add_options()
                    (
                        flag_name.c_str(),
                        boost::program_options::value<int>()->default_value(opt.value()),
                        opt.description().c_str()
                    );
                }
                else if constexpr (std::is_same_v<T, float>)
                {
                    desc.add_options()
                    (
                        flag_name.c_str(),
                        boost::program_options::value<float>()->default_value(opt.value()),
                        opt.description().c_str()
                    );
                }
                else if constexpr (std::is_same_v<T, double>)
                {
                    desc.add_options()
                    (
                        flag_name.c_str(),
                        boost::program_options::value<double>()->default_value(opt.value()),
                        opt.description().c_str()
                    );
                }
                else if constexpr (std::is_same_v<T, std::filesystem::path>)
                {
                    desc.add_options()
                    (
                        flag_name.c_str(),
                        boost::program_options::value<std::string>()->default_value(opt.value().string()),
                        opt.description().c_str()
                    );
                }
                else if constexpr (std::is_same_v<T, cv::Size> || std::is_same_v<T, cv::Scalar>)
                {
                    // Skip complex OpenCV types - only configurable via YAML
                }
                // Add more type handlers here as needed
            };

            // Use all_options() to automatically add all fields
            std::apply([&add_option](const auto&... options)
            {
                (add_option(options), ...);
            }, opts.all_options());

            return desc;
        }

        ~options_base() = default; // Protected destructor for CRTP base
    };
} // namespace zenslam







