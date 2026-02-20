#pragma once

#include <filesystem>

#include <boost/program_options/options_description.hpp>

#include <spdlog/common.h>

#include "zenslam/io/folder_options.h"
#include "zenslam/gui_options.h"
#include "slam_options.h"
#include "zenslam/io/verb.h"

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

        ZENSLAM_DEFINE_OPTIONS
        (
            ((std::filesystem::path, file, "options.yaml", "Path to the options YAML file"))
            ((spdlog::level::level_enum, log_level, spdlog::level::info, "Logging verbosity level"))
            ((std::string, log_pattern, "[%Y-%b-%d %T.%e] [%^%l%$] %v", "Logging pattern"))
            ((verb, verb, verb::RUN, ""))
            ((folder_options, folder, {}, "Folder and file path options"))
            ((gui_options, gui, {}, "GUI and visualization configuration"))
            ((slam_options, slam, {}, "SLAM algorithm configuration options"))
        )

        void validate() const;
        void print() const;
    };

#undef OPTION
} // namespace zenslam
