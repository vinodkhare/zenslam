#pragma once

#include <filesystem>
#include <fstream>

#include "zenslam/frame/system.h"

namespace zenslam::frame
{
    class writer
    {
    public:
        explicit writer(const std::filesystem::path& path);
        auto     write(system& frame) -> void;

    private:
        std::ofstream _file { "frames.csv" };
    };
}