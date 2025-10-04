#include "viewer.h"

#include <opencv2/highgui.hpp>

#include <spdlog/spdlog.h>

#include "stereo_frame.h"
#include "utils.h"

namespace zenslam
{
    viewer::viewer() :
        _thread(&viewer::loop, this)
    {
    }

    viewer::~viewer()
    {
        _is_running = false;
        _cv.notify_all();
        if (_thread.joinable()) _thread.join();
    }

    void viewer::enqueue(const stereo_frame &frame)
    {
        {
            std::lock_guard lock_guard(_mutex);
            _queue.push(frame);
        }

        _cv.notify_one();
    }

    void viewer::loop()
    {
        SPDLOG_INFO("viewer thread started");

        _is_running = true;

        while (_is_running)
        {
            stereo_frame frame;
            {
                std::unique_lock unique_lock(_mutex);

                _cv.wait
                (
                    unique_lock,

                    [this]
                    {
                        return !_is_running || !_queue.empty();
                    }
                );

                if (!_is_running) break;
                if (_queue.empty()) continue; // spurious

                frame = _queue.front();
                _queue.pop();
            }

            cv::namedWindow("L", cv::WINDOW_GUI_EXPANDED);
            cv::imshow("L", frame.l.image);
            cv::setWindowTitle
                    ("L", std::format("L: {{ t: {} }}", utils::to_string_epoch(frame.l.timestamp)));

            cv::namedWindow("R", cv::WINDOW_GUI_EXPANDED);
            cv::imshow("R", frame.r.image);
            cv::setWindowTitle
                    ("R", std::format("R: {{ t: {} }}", utils::to_string_epoch(frame.r.timestamp)));

            cv::waitKey(1);
        }

        SPDLOG_INFO("viewer thread exiting");
    }
} // namespace zenslam
