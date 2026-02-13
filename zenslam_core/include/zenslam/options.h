#pragma once

#include <filesystem>

#include <boost/program_options/options_description.hpp>

#include <spdlog/common.h>

#include "folder_options.h"
#include "slam_options.h"
#include "verb.h"

#include "zenslam/option.h"

namespace zenslam
{
    class options
    {
    public:
        using options_description = boost::program_options::options_description;

        static options_description description();

        static options parse(int argc, char** argv);
        static options parse(const std::filesystem::path& path);

        ZENSLAM_OPTION(std::filesystem::path, file, "options.yaml", "Path to the options YAML file");
        ZENSLAM_OPTION(spdlog::level::level_enum, log_level, spdlog::level::trace, "Logging level");
        ZENSLAM_OPTION(std::string, log_pattern, "[%Y-%b-%d %T.%e] [%^%l%$] %v", "Logging pattern");
        ZENSLAM_OPTION(verb, verb, verb::RUN, "Verbosity level");

        folder_options folder;
        slam_options slam;

        void validate() const;
        void print() const;
    };

#undef OPTION
} // namespace zenslam
