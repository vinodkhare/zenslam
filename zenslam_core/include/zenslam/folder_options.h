#pragma once

#include <filesystem>

namespace zenslam
{
    class folder_options
    {
    public:
        std::filesystem::path folder_root      = ".";
        std::filesystem::path folder_left      = "cam0";
        std::filesystem::path folder_right     = "cam1";
        double                folder_timescale = 1E-9;

        void print() const;
    };
}