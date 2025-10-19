#pragma once

#include <filesystem>
#include <fstream>

#include "zenslam/frame/slam.h"

namespace zenslam::frame
{
    class writer
    {
    public:
        explicit writer(const std::filesystem::path& path);
        auto     write(slam& frame) -> void;

    private:
        std::ofstream _file { "frames.csv" };
    };
}