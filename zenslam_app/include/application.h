#pragma once

#include <memory>
#include <mutex>
#include <set>
#include <vector>

#include <opencv2/core.hpp>

#include <zenslam/options.h>
#include "zenslam/reader_thread.h"
#include "zenslam/slam_thread.h"

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
        // Render helpers (split from render for clarity)
        static bool is_renderable(const frame::system& system);
        static void draw_spatial_matches(const frame::system& system);
        void        draw_temporal_matches(const frame::system& system) const;
        void        draw_scene_vtk(const frame::system& system);
        void        draw_viz_controls();

        // VTK scene container (defined in .cpp to avoid leaking VTK headers)
        struct SceneVTK;

        struct SceneVTKDeleter
        {
            void operator()(const SceneVTK* p) const;
        };

        std::unique_ptr<SceneVTK, SceneVTKDeleter> _vtk;

        std::mutex    _mutex  = {};
        frame::system _system = {};

        options          _options       = {};
        slam_thread      _slam_thread   = slam_thread{_options};
        reader_thread    _reader_thread = reader_thread{_options.folder};
        std::set<size_t> _line_indices  = {};

    // Frame timing history for plots
    std::vector<double> _time_history       = {};
    std::vector<double> _wait_history       = {};
    std::vector<double> _processing_history = {};
    std::vector<double> _tracking_history   = {};
    std::vector<double> _estimation_history = {};
    std::vector<double> _total_history      = {};

    // Frame counts history for plots
    std::vector<double> _kp_l_history          = {};
    std::vector<double> _kp_r_history          = {};
    std::vector<double> _kp_tracked_l_history  = {};
    std::vector<double> _kp_tracked_r_history  = {};
    std::vector<double> _kp_new_l_history      = {};
    std::vector<double> _kp_new_r_history      = {};
    std::vector<double> _kp_total_history      = {};
    std::vector<double> _matches_history       = {};
    std::vector<double> _triangulated_history  = {};
    std::vector<double> _map_points_history    = {};

    // Trajectory history for visualization
    std::vector<cv::Point3d> _trajectory_estimated = {};
    std::vector<cv::Point3d> _trajectory_gt        = {};
    };
} // namespace zenslam
