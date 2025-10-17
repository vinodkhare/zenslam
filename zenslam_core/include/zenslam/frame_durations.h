#pragma once

#include <chrono>

namespace zenslam
{
    struct frame_durations
    {
        std::chrono::system_clock::duration preprocessing = { };
        std::chrono::system_clock::duration tracking      = { };
        std::chrono::system_clock::duration detection     = { };
        std::chrono::system_clock::duration matching      = { };
        std::chrono::system_clock::duration estimation    = { };
        std::chrono::system_clock::duration total         = { };

        auto print() const -> void;
    };

}
