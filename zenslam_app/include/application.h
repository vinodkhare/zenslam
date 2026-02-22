#pragma once

#include <memory>
#include <mutex>
#include <set>
#include <vector>

#include "window.h"

#include "zenslam/all_options.h"
#include "zenslam/slam_thread.h"
#include "zenslam/io/reader_thread.h"

namespace zenslam
{
    class application
    {
    public:
        explicit application(all_options options);
        ~application();

        void render();

    private:
        // Check if system has renderable data
        static bool is_renderable(const frame::system& system);

        std::mutex    _mutex  = { };
        frame::system _system = { };

        all_options      _options       = { };
        slam_thread      _slam_thread   = slam_thread { _options };
        reader_thread    _reader_thread = reader_thread { _options.folder };
        std::set<size_t> _line_indices  = { };

        // Trajectory history for visualization (shared with VTK window)
        std::vector<cv::Point3d> _trajectory_estimated = { };
        std::vector<cv::Point3d> _trajectory_gt        = { };

        // Window instances
        std::vector<std::shared_ptr<window>> _windows = { };
    };
} // namespace zenslam
