#pragma once

#include <functional>
#include <map>
#include <string>

#include <boost/program_options/parsers.hpp>

namespace zenslam::utils
{
    inline std::string version = "0.0.1";

    std::string epoch_double_to_string(double epoch_seconds);

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

}
