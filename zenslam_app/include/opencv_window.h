#pragma once

#include <string>

#include <zenslam/gui_options.h>

#include "window.h"

namespace zenslam
{
    /**
     * @brief OpenCV-based window for displaying 2D image visualizations.
     *
     * This class wraps OpenCV's imshow functionality to display images
     * such as spatial and temporal feature matches.
     */
    class opencv_window : public window
    {
    public:
        /**
         * @brief Window type enumeration for different OpenCV visualization modes.
         */
        enum class type
        {
            spatial_matches, ///< Display spatial (stereo) feature matches
            temporal_matches ///< Display temporal (frame-to-frame) feature matches
        };

        /**
         * @brief Construct an OpenCV window of the specified type.
         *
         * @param window_type The type of visualization this window will display.
         * @param gui_options Reference to GUI options (needed for temporal matches rendering).
         */
        opencv_window(type window_type, const gui_options& gui_options);

        void               initialize() override;
        void               render(const frame::system& system) override;
        [[nodiscard]] bool is_initialized() const override { return _initialized; }
        void               set_visible(const bool visible) override { _visible = visible; }
        [[nodiscard]] bool is_visible() const override { return _visible; }

    private:
        type              _type = type::spatial_matches;
        const gui_options& _gui_options;
        std::string       _window_name   = { };
        std::string    _window_title  = { };
        int            _window_width  = 1024;
        int            _window_height = 512;
        bool           _initialized   = false;
        bool           _visible       = true;
    };
} // namespace zenslam
