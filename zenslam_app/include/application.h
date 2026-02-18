#pragma once

#include <memory>
#include <mutex>
#include <set>
#include <vector>

#include <opencv2/core.hpp>

#include <zenslam/options.h>
#include "zenslam/io/reader_thread.h"
#include "zenslam/slam_thread.h"
#include "window.h"

namespace zenslam
{
    class options;

    class application
    {
    public:
        explicit application(options options);
        ~application();

        void render();

    private:
        // Check if system has renderable data
        static bool is_renderable(const frame::system& system);

        std::mutex    _mutex  = { };
        frame::system _system = { };

        options          _options       = { };
        slam_thread      _slam_thread   = slam_thread { _options };
        reader_thread    _reader_thread = reader_thread { _options.folder };
        std::set<size_t> _line_indices  = { };

        // Trajectory history for visualization (shared with VTK window)
        std::vector<cv::Point3d> _trajectory_estimated = { };
        std::vector<cv::Point3d> _trajectory_gt        = { };

        // Visualization controls (shared with ImGui window)
        float _point_cloud_opacity = 1.0f;
        float _point_size          = 4.0f;

        // Window instances
        std::vector<std::shared_ptr<window>> _windows = { };
    };
} // namespace zenslam
