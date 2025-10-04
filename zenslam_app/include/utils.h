#pragma once

#include <functional>
#include <map>
#include <string>

#include <boost/program_options/parsers.hpp>

#include <spdlog/common.h>

#include "stereo_frame.h"

namespace zenslam::utils
{
    template<typename T_KEY, typename T_VALUE>
    auto invert(const std::map<T_KEY, T_VALUE> &map) -> std::map<T_VALUE, T_KEY>
    {
        auto inverted = std::map<T_VALUE, T_KEY> { };

        for (auto &pair: map)
        {
            inverted[pair.second] = pair.first;
        }

        return inverted;
    }

    template<typename T>
    std::map<std::string, T> to_map(const std::vector<T> &parsed)
    {
        auto parsed_map = std::map<std::string, T> { };

        for (auto &option: parsed)
        {
            parsed_map[option.string_key] = option;
        }

        return parsed_map;
    }

    inline std::string version = "0.0.1";

    inline std::map<std::string, spdlog::level::level_enum> log_levels_from_string =
    {
        { "trace", spdlog::level::trace },
        { "debug", spdlog::level::debug },
        { "info", spdlog::level::info },
        { "warn", spdlog::level::warn },
        { "error", spdlog::level::err },
        { "critical", spdlog::level::critical },
        { "off", spdlog::level::off }
    };

    inline auto log_levels_to_string = invert(log_levels_from_string);

    auto        draw_keypoints(const mono_frame &frame) -> cv::Mat;
    auto        draw_matches(const stereo_frame &frame) -> cv::Mat;
    std::string epoch_double_to_string(double epoch_seconds);
}
