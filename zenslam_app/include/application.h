#pragma once

#include <opencv2/viz/viz3d.hpp>

#include <zenslam/options.h>
#include <zenslam/slam_thread.h>

namespace zenslam
{
    class application
    {
    public:
        explicit application(options options);

        void render();

    private:
        std::mutex  _mutex { };
        frame::slam _slam { };

        options                         _options { };
        slam_thread                     _slam_thread { _options };
        std::unique_ptr<cv::viz::Viz3d> _viewer { };
        cv::viz::WWidgetMerger          _merger       = { };
        std::set<size_t>                _line_indices = { };

        // UI state for checkboxes
        bool _show_keypoints { true };
        bool _show_keylines { true };
    };
}
