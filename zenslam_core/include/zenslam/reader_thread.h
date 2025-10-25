#pragma once

#include <thread>
#include <utility>

#include <spdlog/spdlog.h>

#include "time_this.h"

#include "zenslam/options.h"
#include "zenslam/stereo_folder_reader.h"
#include "zenslam/frame/sensor.h"
#include "zenslam/types/event.h"

namespace zenslam
{
    class reader_thread
    {
    public:
        event<frame::sensor> on_frame = {};

        explicit reader_thread(class options::folder options, size_t batch_size = 8, size_t num_threads = 4) :
            _options{std::move(options)},
            _batch_size{batch_size},
            _num_threads{num_threads}
        {
        }

        ~reader_thread()
        {
            _stop_source.request_stop();
        }

    private:
        class options::folder _options     = {};
        size_t                _batch_size  = 4;
        size_t                _num_threads = 4;

        std::stop_source _stop_source = {};
        std::stop_token  _stop_token  = {_stop_source.get_token()};
        std::jthread     _thread      = std::jthread{&reader_thread::loop, this};

        void loop()
        {
            auto       reader = stereo_folder_reader(_options);
            const auto total  = reader.size();


            std::chrono::system_clock::duration batch_time = {};
            {
                time_this time_this{batch_time};

                // Read frames sequentially for this batch
                for (size_t i = 0; i < total; ++i)
                {
                    if (_stop_token.stop_requested()) break;

                    auto frame = reader.read();
                    on_frame(std::move(frame));
                }
            }
        }
    };
}
