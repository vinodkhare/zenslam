#pragma once

#include <opencv2/viz/viz3d.hpp>
#include <zenslam/slam_thread.h>
#include <zenslam/options.h>
#include <zenslam/stereo_frame.h>

namespace zenslam
{
    class application
    {
    public:
        explicit application(options options);

        void render();

    private:
        std::mutex _mutex { };
        slam_frame _slam { };

        options                         _options { };
        slam_thread                     _slam_thread { _options };
        std::unique_ptr<cv::viz::Viz3d> _viewer { };
    };
}
