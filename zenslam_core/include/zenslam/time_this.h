#pragma once

#include <chrono>

namespace zenslam
{
    class time_this
    {
    public:
        time_this()                            = delete;
        time_this(const time_this&)            = delete;
        time_this(time_this&&)                 = delete;
        time_this& operator=(const time_this&) = delete;
        time_this& operator=(time_this&&)      = delete;

        explicit time_this(std::chrono::system_clock::duration& time) :
            _time { time }
        {
        }

        ~time_this()
        {
            _time = std::chrono::system_clock::now() - _start;
        }

    private:
        std::chrono::time_point<std::chrono::system_clock> _start = std::chrono::system_clock::now();
        std::chrono::system_clock::duration&               _time;
    };
}