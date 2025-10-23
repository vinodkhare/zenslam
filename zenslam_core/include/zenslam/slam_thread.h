#pragma once

#include <thread>

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

    private:
        options _options { };

        std::stop_source _stop_source { };
        std::stop_token  _stop_token { _stop_source.get_token() };
        std::jthread     _thread { &slam_thread::loop, this };

        void loop();
    };
}
