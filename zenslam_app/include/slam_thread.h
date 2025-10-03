#pragma once

#include <thread>

#include "event.h"
#include "options.h"
#include "stereo_frame.h"

namespace zenslam
{
    class slam_thread
    {
    public:
        event<stereo_frame> on_frame;

        explicit slam_thread(options options);
        ~slam_thread();

    private:
        options _options { };

        std::stop_source _stop_source { };
        std::stop_token _stop_token { _stop_source.get_token() };
        std::jthread    _thread { &slam_thread::loop, this };

        void loop();
    };
}
