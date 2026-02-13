#pragma once

#include <zenslam/frame/system.h>

namespace zenslam
{
    /**
     * @brief Abstract base class for all window types in the application.
     *
     * This interface provides a common contract for different window implementations
     * (OpenCV imshow, ImGui, VTK) to allow for uniform handling in the application.
     */
    class window
    {
    public:
        virtual ~window() = default;

        /**
         * @brief Initialize the window and allocate necessary resources.
         *
         * This method is called once before the first render. Implementations
         * should create window instances, set up rendering pipelines, etc.
         */
        virtual void initialize() = 0;

        /**
         * @brief Render the window with the given frame data.
         *
         * @param system The current SLAM system frame containing all state information.
         *
         * This method is called every frame. Implementations should update their
         * visualization based on the provided system state.
         */
        virtual void render(const frame::system& system) = 0;

        /**
         * @brief Check if the window has been initialized.
         *
         * @return true if initialize() has been called, false otherwise.
         */
        [[nodiscard]] virtual bool is_initialized() const = 0;

        /**
         * @brief Set the visibility of the window.
         *
         * @param visible true to show the window, false to hide it.
         */
        virtual void set_visible(bool visible) = 0;

        /**
         * @brief Check if the window is visible.
         *
         * @return true if the window is visible, false otherwise.
         */
        [[nodiscard]] virtual bool is_visible() const = 0;
    };
} // namespace zenslam
