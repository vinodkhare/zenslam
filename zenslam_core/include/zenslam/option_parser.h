#pragma once

#include "zenslam/option.h"
#include "zenslam/folder_options.h"

#include <yaml-cpp/yaml.h>
#include <filesystem>

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

        /** YAML parser for std::filesystem::path */
        static auto parse_yaml(const option<std::filesystem::path>& option, const YAML::Node& node) -> zenslam::option<std::filesystem::path>
        {
            zenslam::option<std::filesystem::path> result = option;

            if (const auto n = node[option.name()])
            {
                result = n.template as<std::string>();
            }

            return result;
        }

        template <typename E> requires std::is_enum_v<E>
        static auto parse_yaml(const option<E>& option, const YAML::Node& node) -> zenslam::option<E>
        {
            zenslam::option<E> result = option;
            
            if (const auto n = node[option.name()])
            {
                if (auto v = magic_enum::enum_cast<E>(n.template as<std::string>()))
                {
                    result = v.value();
                }
                else 
                {
                    throw std::runtime_error("Invalid enum value for " + option.name() + ": " + n.template as<std::string>());
                }
            }

            return result;
        };
    };
}