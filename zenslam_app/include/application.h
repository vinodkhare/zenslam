#pragma once

#include <opencv2/viz/viz3d.hpp>
#include <zenslam/slam_thread.h>
#include <zenslam/options.h>
#include <zenslam/stereo_frame.h>
#include <zenslam/thread_safe.h>

namespace zenslam
{
    class application
    {
    public:
        explicit application(const options &options);

        void render();

    private:
        thread_safe<stereo_frame> _frame_1 { };
        thread_safe<stereo_frame> _frame_0 { };

        options     _options { };
        slam_thread _slam_thread { _options };
        std::unique_ptr<cv::viz::Viz3d> _viewer {};
    };
}
