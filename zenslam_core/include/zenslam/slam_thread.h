#pragma once

#include <thread>

#include <opencv2/core/affine.hpp>

#include "event.h"
#include "options.h"
#include "stereo_folder_reader.h"
#include "frame/slam.h"
#include "frame/stereo.h"

namespace zenslam
{
    class slam_thread
    {
    public:
        event<zenslam::frame::slam> on_frame;

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