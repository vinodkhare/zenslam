#pragma once

#include "zenslam/options.h"
#include "zenslam/reader_thread.h"
#include "zenslam/slam_thread.h"

#include <memory>
#include <set>
#include <mutex>

namespace zenslam
{
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
        void draw_temporal_matches(const frame::system& system) const;
        void draw_scene_vtk(const frame::system& system);
        void draw_viz_controls();

        // VTK scene container (defined in .cpp to avoid leaking VTK headers)
        struct SceneVTK;
        struct SceneVTKDeleter { void operator()(const SceneVTK* p) const; };
        std::unique_ptr<SceneVTK, SceneVTKDeleter> _vtk;

        std::mutex    _mutex  = { };
        frame::system _system = { };

        options         _options       = { };
        slam_thread     _slam_thread   = slam_thread { _options };
        reader_thread   _reader_thread = reader_thread { _options.folder };
        std::set<size_t> _line_indices = { };

        // UI state for checkboxes
        bool _show_keypoints { true };
        bool _show_keylines { true };
    };
}
