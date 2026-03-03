#pragma once

#include <condition_variable>
#include <queue>
#include <thread>

#include "zenslam/all_options.h"
#include "zenslam/frame/system.h"
#include "zenslam/types/event.h"

namespace zenslam
{
    class slam_thread
    {
    public:
        event<frame::system> on_frame;

        explicit slam_thread(all_options options);
        ~slam_thread();

        void enqueue(const frame::sensor& frame);

        void request_stop();

    private:
        all_options               _options = { };
        std::mutex                _mutex   = { };
        std::condition_variable   _cv      = { };
        std::queue<frame::sensor> _queue   = { };

        std::stop_source _stop_source = { };
        std::stop_token  _stop_token  = { _stop_source.get_token() };
        std::jthread     _thread      = std::jthread { &slam_thread::loop, this };

        void loop();
    };
}
