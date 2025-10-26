#pragma once

#include <chrono>

namespace zenslam::frame
{
    struct durations
    {
        std::chrono::system_clock::duration wait       = { };
        std::chrono::system_clock::duration processing = { };
        std::chrono::system_clock::duration tracking   = { };
        std::chrono::system_clock::duration estimation = { };
        std::chrono::system_clock::duration total      = { };

        auto print() const -> void;
    };
}