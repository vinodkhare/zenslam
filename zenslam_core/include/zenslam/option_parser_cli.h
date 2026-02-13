#pragma once

#include <algorithm>
#include <filesystem>
#include <map>
#include <string>

#include <magic_enum/magic_enum.hpp>

#include <opencv2/core/types.hpp>

#include "zenslam/option.h"

namespace zenslam
{
    /**
     * @brief Parser for option<T> from command-line arguments
     *
     * Provides static functions to parse option<T> values from boost::program_options
     * variable map and options map. Works in conjunction with the CLI-based options
     * defined in description() methods.
     *
     * The flag name is constructed by concatenating the prefix with the option's name
     * (with underscores replaced by dashes). For example:
     *   - option name: "root", prefix: "folder-" → flag name: "folder-root"
     *   - option name: "matcher_ratio", prefix: "" → flag name: "matcher-ratio"
     */
    class option_parser_cli
    {
    private:
        /**
         * @brief Helper to construct flag name from option name and prefix
         * @param prefix The prefix to add (e.g., "folder-")
         * @param option_name The option's name field (with underscores)
         * @return The constructed CLI flag name (with dashes)
         */
        static auto construct_flag_name(const std::string& prefix, const std::string& option_name) -> std::string
        {
            std::string flag_name = prefix + option_name;
            // Replace underscores with dashes
            std::replace(flag_name.begin(), flag_name.end(), '_', '-');
            return flag_name;
        }

    public:
        /**
         * @brief Parse an option<T> from CLI arguments
         * @tparam T The type of the option
         * @param option The option to parse into
         * @param prefix The CLI flag prefix (e.g., "folder-", "")
         * @param options_map Map of parsed CLI options
         * @param vm Variables map from boost::program_options
         * @return Parsed option<T> with updated value if flag was provided
         */
        template <typename T>
        static auto parse_cli(
            const option<T>& option,
            const std::string& prefix,
            const std::map<std::string, boost::program_options::basic_option<char>>& options_map,
            const boost::program_options::variables_map& vm) -> zenslam::option<T>
        {
            zenslam::option<T> result = option;

            const auto flag_name = construct_flag_name(prefix, option.name());

            if (options_map.contains(flag_name))
            {
                if (vm.contains(flag_name))
                {
                    result = vm.at(flag_name).template as<T>();
                }
            }

            return result;
        }

        /**
         * @brief Parse option<std::filesystem::path> from CLI arguments
         */
        static auto parse_cli(
            const option<std::filesystem::path>& option,
            const std::string& prefix,
            const std::map<std::string, boost::program_options::basic_option<char>>& options_map,
            const boost::program_options::variables_map& vm) -> zenslam::option<std::filesystem::path>
        {
            zenslam::option<std::filesystem::path> result = option;

            const auto flag_name = construct_flag_name(prefix, option.name());

            if (options_map.contains(flag_name))
            {
                if (vm.contains(flag_name))
                {
                    result = vm.at(flag_name).as<std::string>();
                }
            }

            return result;
        }

        /**
         * @brief Parse option<cv::Size> from CLI arguments
         * Note: cv::Size is typically not exposed as CLI option, only available via YAML
         */
        static auto parse_cli(
            const option<cv::Size>& option,
            const std::string& prefix,
            const std::map<std::string, boost::program_options::basic_option<char>>& options_map,
            const boost::program_options::variables_map& vm) -> zenslam::option<cv::Size>
        {
            // cv::Size not supported via CLI - keep original value
            return option;
        }

        /**
         * @brief Parse option<cv::Scalar> from CLI arguments
         */
        static auto parse_cli(
            const option<cv::Scalar>& option,
            const std::string& prefix,
            const std::map<std::string, boost::program_options::basic_option<char>>& options_map,
            const boost::program_options::variables_map& vm) -> zenslam::option<cv::Scalar>
        {
            zenslam::option<cv::Scalar> result = option;

            const auto flag_name = construct_flag_name(prefix, option.name());

            if (options_map.contains(flag_name))
            {
                if (vm.contains(flag_name))
                {
                    const auto& vec = vm.at(flag_name).as<std::vector<int>>();
                    if (vec.size() >= 3)
                    {
                        // Clamp color values to [0, 255]
                        auto clamp = [](int v) { return std::max(0, std::min(255, v)); };
                        result = cv::Scalar(clamp(vec[0]), clamp(vec[1]), clamp(vec[2]));
                    }
                }
            }

            return result;
        }

        /**
         * @brief Parse option<E> where E is an enum type from CLI arguments
         */
        template <typename E> requires std::is_enum_v<E>
        static auto parse_cli(
            const option<E>& option,
            const std::string& prefix,
            const std::map<std::string, boost::program_options::basic_option<char>>& options_map,
            const boost::program_options::variables_map& vm) -> zenslam::option<E>
        {
            zenslam::option<E> result = option;

            const auto flag_name = construct_flag_name(prefix, option.name());

            if (options_map.contains(flag_name))
            {
                if (vm.contains(flag_name))
                {
                    if (auto v = magic_enum::enum_cast<E>(vm.at(flag_name).template as<std::string>()))
                    {
                        result = v.value();
                    }
                }
            }

            return result;
        }
    };
} // namespace zenslam

