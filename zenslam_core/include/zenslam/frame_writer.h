#pragma once

#include <fstream>

#include "slam_frame.h"

namespace zenslam
{
    class frame_writer
    {
    public:
        explicit frame_writer(const std::filesystem::path &path);

        auto write(slam_frame &frame) -> void;

    private:
        std::ofstream _file { "frames.csv" };
    };
}
