#pragma once

#include "zenslam/option.h"

#include <yaml-cpp/yaml.h>

namespace zenslam
{
    class option_parser
    {
    public:
        template <typename T>
        static auto parse_yaml(const option<T>& option, const YAML::Node& node) -> zenslam::option<T>
        {
            zenslam::option<T> result = option;

            if (const auto n = node[option.name()])
            {
                result = n.template as<T>();
            }

            return result;
        }
    };
}