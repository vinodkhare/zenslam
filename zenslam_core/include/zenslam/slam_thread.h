#pragma once

#include <queue>
#include <thread>

#include <concurrentqueue/moodycamel/concurrentqueue.h>

#include "zenslam/options.h"
#include "zenslam/frame/system.h"
#include "zenslam/types/event.h"

namespace zenslam
{
    class slam_thread
    {
    public:
        event<frame::system> on_frame;

        explicit slam_thread(options options);
        ~slam_thread();

        void enqueue(const frame::sensor& frame);

    private:
        using concurrent_queue = moodycamel::ConcurrentQueue<frame::sensor>;

        options                   _options = { };
        std::mutex                _mutex   = { };
        std::queue<frame::sensor> _queue   = { };

        std::stop_source _stop_source = { };
        std::stop_token  _stop_token  = { _stop_source.get_token() };
        std::jthread     _thread      = std::jthread { &slam_thread::loop, this };

        void loop();
    };
}
