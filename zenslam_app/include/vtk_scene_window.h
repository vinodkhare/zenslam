#pragma once

#include "window.h"
#include <zenslam/options.h>
#include <memory>
#include <vector>

namespace zenslam
{
    /**
     * @brief VTK-based window for 3D scene visualization.
     *
     * This class handles all VTK rendering including:
     * - 3D point cloud of landmarks
     * - 3D line segments
     * - Camera pose axes (estimated and ground truth)
     * - Camera frustum with texture
     * - Trajectory paths (estimated and ground truth)
     */
    class vtk_scene_window : public window
    {
    public:
        /**
         * @brief Construct a VTK scene window.
         *
         * @param options Reference to SLAM options for controlling visualization.
         * @param point_cloud_opacity Reference to point cloud opacity (controlled by ImGui).
         * @param trajectory_estimated Reference to estimated trajectory history.
         * @param trajectory_gt Reference to ground truth trajectory history.
         */
        vtk_scene_window(const options&            options,
                         float&                    point_cloud_opacity,
                         std::vector<cv::Point3d>& trajectory_estimated,
                         std::vector<cv::Point3d>& trajectory_gt);

        ~vtk_scene_window() override;

        void               initialize() override;
        void               render(const frame::system& system) override;
        [[nodiscard]] bool is_initialized() const override { return _initialized; }
        void               set_visible(bool visible) override { _visible = visible; }
        [[nodiscard]] bool is_visible() const override { return _visible; }

        /**
         * @brief Set the background color(s) for the VTK window.
         * @param r1 Red component of bottom color (0.0 - 1.0)
         * @param g1 Green component of bottom color (0.0 - 1.0)
         * @param b1 Blue component of bottom color (0.0 - 1.0)
         * @param r2 Red component of top color (0.0 - 1.0), defaults to r1
         * @param g2 Green component of top color (0.0 - 1.0), defaults to g1
         * @param b2 Blue component of top color (0.0 - 1.0), defaults to b1
         */
        void set_background_color(double r1, double g1, double b1, 
                                  double r2 = -1.0, double g2 = -1.0, double b2 = -1.0);

    private:
        const options&            _options;
        float&                    _point_cloud_opacity;
        std::vector<cv::Point3d>& _trajectory_estimated;
        std::vector<cv::Point3d>& _trajectory_gt;
        bool                      _initialized = false;
        bool                      _visible     = true;

        // VTK scene container (forward declared to avoid leaking VTK headers)
        struct scene_vtk;

        struct scene_vtk_deleter
        {
            void operator()(const scene_vtk* p) const;
        };

        std::unique_ptr<scene_vtk, scene_vtk_deleter> _scene;
    };
} // namespace zenslam
