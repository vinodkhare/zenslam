#include "application.h"

#include <opencv2/highgui.hpp>

#include "opencv_window.h"
#include "imgui_controls_window.h"
#include "vtk_scene_window.h"

zenslam::application::application(options options) :
    _options { std::move(options) }
{
    // Create window instances
    _windows.push_back(std::make_shared<opencv_window>(opencv_window::type::spatial_matches, _options.slam->gui));
    _windows.push_back(std::make_shared<opencv_window>(opencv_window::type::temporal_matches, _options.slam->gui));
    _windows.push_back(std::make_shared<vtk_scene_window>(_options, _options.slam->gui, _trajectory_estimated, _trajectory_gt));

    // Create ImGui controls window and get reference for history updates
    auto imgui_window = std::make_shared<imgui_controls_window>(_options.slam->gui);
    _windows.push_back(imgui_window);

    // Set up frame callback
    _slam_thread.on_frame += [this, imgui_window](const frame::system& frame)
    {
        std::lock_guard lock { _mutex };
        _system = frame;

        // Update trajectory history
        {
            const auto& pos = _system[1].pose.translation();
            _trajectory_estimated.emplace_back(pos[0], pos[1], pos[2]);

            const auto& pos_gt = _system[1].pose_gt.translation();
            _trajectory_gt.emplace_back(pos_gt[0], pos_gt[1], pos_gt[2]);
        }

        // Update ImGui window history
        imgui_window->update_history(_system);
    };

    _reader_thread.on_frame += [this](const frame::sensor& frame) { _slam_thread.enqueue(frame); };
}

zenslam::application::~application() = default;

void zenslam::application::render()
{
    frame::system system { };
    {
        std::lock_guard lock { _mutex };
        system = _system;
    }

    if (!is_renderable(system))
        return;

    // Render all windows
    for (auto& window : _windows)
    {
        if (window->is_visible())
        {
            window->render(system);
        }
    }

    cv::waitKey(1);
}

bool zenslam::application::is_renderable(const frame::system& system)
{
    // Require undistorted images and some keypoints to render informative views
    return !system[0].undistorted[0].empty() && !system[1].undistorted[0].empty() && !system[1].keypoints[0].empty();
}
