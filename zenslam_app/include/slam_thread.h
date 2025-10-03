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
        event<stereo_frame> on_keypoints;

        explicit slam_thread(const options &options);

    private:
        options _options { };

        std::jthread _thread { &slam_thread::loop, this };

        void loop();
    };
}
