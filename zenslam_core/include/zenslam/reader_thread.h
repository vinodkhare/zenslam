#pragma once

#include <thread>

#include "zenslam/options.h"
#include "zenslam/stereo_folder_reader.h"
#include "zenslam/frame/sensor.h"
#include "zenslam/types/event.h"

namespace zenslam
{
    class reader_thread
    {
    public:
        event<frame::sensor> on_frame = { };

        explicit reader_thread(const class options::folder& options) :
            _options { options } {}

        ~reader_thread()
        {
            _stop_source.request_stop();
        }

    private:
        class options::folder _options = { };

        std::stop_source _stop_source = { };
        std::stop_token  _stop_token  = { _stop_source.get_token() };
        std::jthread     _thread      = std::jthread { &reader_thread::loop, this };

        void loop()
        {
            const auto reader = stereo_folder_reader(_options);

            for (auto stereo: reader)
            {
                on_frame({ frame::sensor::count++, stereo.cameras[0].timestamp, { stereo.cameras[0].image, stereo.cameras[1].image } });

                if (_stop_token.stop_requested()) break;
            }
        }
    };
}
