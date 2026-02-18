#pragma once

#include <vector>

#include <zenslam/options.h>

#include "window.h"

namespace zenslam
{
    /**
     * @brief ImGui-based window for displaying UI controls and plots.
     *
     * This class handles all ImGui/ImPlot visualizations including:
     * - Current frame information (pose, IMU data)
     * - Visualization controls (checkboxes, sliders, color pickers)
     * - Performance plots (frame timing)
     * - Feature count plots (keypoints, keylines)
     */
    class imgui_controls_window : public window
    {
    public:
        /**
         * @brief Construct an ImGui controls window.
         *
         * @param options Reference to SLAM options for controlling visualization settings.
         * @param point_cloud_opacity Reference to point cloud opacity (shared with VTK window).
         * @param point_size Reference to point cloud point size (shared with VTK window).
         */
        imgui_controls_window(options& options, float& point_cloud_opacity, float& point_size);

        void               initialize() override;
        void               render(const frame::system& system) override;
        [[nodiscard]] bool is_initialized() const override { return _initialized; }
        void               set_visible(bool visible) override { _visible = visible; }
        [[nodiscard]] bool is_visible() const override { return _visible; }

        /**
         * @brief Update history data for plotting.
         *
         * This method should be called whenever a new frame is processed to update
         * the historical data used for time-series plots.
         *
         * @param system The current SLAM system frame.
         */
        void update_history(const frame::system& system);

    private:
        options& _options;
        float&   _point_cloud_opacity;
        float&   _point_size;
        bool     _initialized = false;
        bool     _visible     = true;

        // Frame timing history for plots
        std::vector<double> _time_history       = { };
        std::vector<double> _wait_history       = { };
        std::vector<double> _processing_history = { };
        std::vector<double> _tracking_history   = { };
        std::vector<double> _estimation_history = { };
        std::vector<double> _total_history      = { };

        // Feature count history structure
        struct feature_count_history
        {
            std::vector<double> features_l         = { };
            std::vector<double> features_r         = { };
            std::vector<double> features_l_tracked = { };
            std::vector<double> features_r_tracked = { };
            std::vector<double> features_l_new     = { };
            std::vector<double> features_r_new     = { };
            std::vector<double> features_total     = { }; // Only used for points
            std::vector<double> matches_stereo     = { };
            std::vector<double> triangulated_3d    = { };
            std::vector<double> map_total          = { };
        };

        // Feature count history instances
        feature_count_history _point_history = { };
        feature_count_history _line_history  = { };
    };
} // namespace zenslam
