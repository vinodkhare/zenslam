#pragma once

#include <thread>
#include <future>
#include <vector>

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
        event<frame::sensor> on_frame = { };

        explicit reader_thread(const class options::folder& options, size_t batch_size = 4, size_t num_threads = 4) :
            _options { options },
            _batch_size { batch_size },
            _num_threads { num_threads } {}

        ~reader_thread()
        {
            _stop_source.request_stop();
        }

    private:
        class options::folder _options      = { };
        size_t                _batch_size   = 4;
        size_t                _num_threads  = 4;

        std::stop_source _stop_source = { };
        std::stop_token  _stop_token  = { _stop_source.get_token() };
        std::jthread     _thread      = std::jthread { &reader_thread::loop, this };

        void loop()
        {
            const auto reader = stereo_folder_reader(_options);
            const auto total  = reader.size();

            for (size_t batch_start = 0; batch_start < total; batch_start += _batch_size)
            {
                if (_stop_token.stop_requested()) break;

                const size_t batch_end = std::min(batch_start + _batch_size, total);
                const size_t batch_count = batch_end - batch_start;

                std::chrono::system_clock::duration batch_time = { };
                {
                    time_this time_this { batch_time };

                    // Launch parallel reads for this batch
                    std::vector<std::future<frame::sensor>> futures;
                    futures.reserve(batch_count);

                    for (size_t i = batch_start; i < batch_end; ++i)
                    {
                        futures.push_back
                        (
                            std::async
                            (
                                std::launch::async,
                                [&reader, i]()
                                {
                                    return reader[i];
                                }
                            )
                        );
                    }

                    // Collect results in order and publish
                    for (auto& future : futures)
                    {
                        if (_stop_token.stop_requested()) break;

                        on_frame(future.get());
                    }
                }

                const double batch_time_s = std::chrono::duration<double>(batch_time).count();
                const double per_frame_s = batch_time_s / batch_count;
                SPDLOG_INFO("batch read: {} frames in {:.4f} s ({:.4f} s/frame)", batch_count, batch_time_s, per_frame_s);
            }
        }
    };
}
