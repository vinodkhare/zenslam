#pragma once

#include <chrono>

namespace zenslam::frame
{
    struct durations
    {
        std::chrono::system_clock::duration wait            = { };
        std::chrono::system_clock::duration processing      = { };
        std::chrono::system_clock::duration tracking        = { };
        std::chrono::system_clock::duration estimation      = { };
        std::chrono::system_clock::duration total           = { };
        
        // Fine-grained timing
        std::chrono::system_clock::duration detection_left  = { };
        std::chrono::system_clock::duration detection_right = { };
        std::chrono::system_clock::duration matching        = { };
        std::chrono::system_clock::duration triangulation   = { };
        std::chrono::system_clock::duration klt             = { };

        auto print() const -> void;
    };
}