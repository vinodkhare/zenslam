#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>

#include "stereo_frame.h"

namespace zenslam
{
    class viewer
    {
    public:
        viewer();
        ~viewer();

        viewer(const viewer &)                = delete;
        viewer &operator=(const viewer &)     = delete;
        viewer(viewer &&) noexcept            = delete;
        viewer &operator=(viewer &&) noexcept = delete;

        // Enqueue a frame to be displayed by the background thread
        void enqueue(const stereo_frame &frame);

    private:
        std::queue<stereo_frame> _queue {}; // FIFO of frames
        std::mutex               _mutex {}; // protects queue
        std::condition_variable  _cv {};    // signals arrival
        std::atomic<bool>        _is_running { true };
        std::thread              _thread {}; // background consumer

        void loop();
    };
} // namespace zenslam
