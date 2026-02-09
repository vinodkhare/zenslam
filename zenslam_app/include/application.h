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

    // Feature count history structure
    struct feature_count_history {
        std::vector<double> features_l          = {};
        std::vector<double> features_r          = {};
        std::vector<double> features_l_tracked  = {};
        std::vector<double> features_r_tracked  = {};
        std::vector<double> features_l_new      = {};
        std::vector<double> features_r_new      = {};
        std::vector<double> features_total      = {};  // Only used for points
        std::vector<double> matches_stereo      = {};
        std::vector<double> triangulated_3d     = {};
        std::vector<double> map_total           = {};
    };

    // Feature count history instances
    feature_count_history _point_history = {};
    feature_count_history _line_history  = {};

    // Trajectory history for visualization
    std::vector<cv::Point3d> _trajectory_estimated = {};
    std::vector<cv::Point3d> _trajectory_gt        = {};

    // Visualization controls
    float _point_cloud_opacity = 1.0f;
    };
} // namespace zenslam
