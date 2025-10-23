#include "application.h"

#include <imgui.h>

#include <opencv2/highgui.hpp>
#include <opencv2/viz.hpp>

#include <opencv2/viz/widgets.hpp>
#include <zenslam/utils.h>

#include <numbers>
#include <utility>

#include "zenslam/utils_opencv.h"

zenslam::application::application(options options) :
    _options { std::move(options) },
    _show_keypoints { _options.slam.show_keypoints },
    _show_keylines { _options.slam.show_keylines }
{
    _slam_thread.on_frame += [this](const frame::system& slam)
    {
        std::lock_guard lock { _mutex };

        _system = slam;
    };
}

void zenslam::application::render()
{
    frame::system system { };
    {
        std::lock_guard lock { _mutex };
        system = _system;
    }

    if (system[1].keypoints[0].empty() || system[0].undistorted[0].empty() || system[1].undistorted[0].empty()) return;

    // display matches spatial
    {
        const auto& matches_image = utils::draw_matches_spatial(system[1], system.points3d);

        cv::imshow("matches_spatial", matches_image);
        cv::setWindowTitle("matches_spatial", "matches spatial");
        cv::resizeWindow("matches_spatial", 1024, 512);
    }

    // display matches temporal
    {
        const auto& image = utils::draw_matches_temporal
        (
            system[0],
            system[1],
            _options.slam
        );

        cv::namedWindow("matches_temporal");
        cv::imshow("matches_temporal", image);
        cv::setWindowTitle("matches_temporal", "matches temporal");
        cv::resizeWindow("matches_temporal", 1024, 512);
    }

    // display 3D points using viz module
    {
        if (!_viewer)
        {
            _viewer = std::make_unique<cv::viz::Viz3d>("3D Points");

            _viewer->setBackgroundColor();
            _viewer->setWindowSize(cv::Size(1024, 1024));   // TODO: make this configurable
        }
        else if (!system.points3d.empty())
        {
            _viewer->removeAllWidgets();

            _viewer->showWidget("origin", cv::viz::WCoordinateSystem());

            cv::viz::WCameraPosition camera_position
            (
                cv::Vec2d { std::numbers::pi / 2, std::numbers::pi / 2 },
                system[1].undistorted[0]
            );

            camera_position.setColor(cv::viz::Color::red());
            _viewer->showWidget("camera", camera_position);     // TODO: fix camera size
            _viewer->setWidgetPose("camera", system[1].pose);

            cv::viz::WCameraPosition camera_gt
            (
                cv::Vec2d { std::numbers::pi / 2, std::numbers::pi / 2 },
                system[1].undistorted[0]
            );

            camera_gt.setColor(cv::viz::Color::bluberry());
            _viewer->showWidget("camera_gt", camera_gt);
            _viewer->setWidgetPose("camera_gt", system[1].pose_gt);

            const auto& points = system.points3d.values_cast<cv::Point3d>() | std::ranges::to<std::vector>();

            _viewer->showWidget("cloud", cv::viz::WCloud(points));
            _viewer->setRenderingProperty("cloud", cv::viz::POINT_SIZE, 4.0);

            if (_options.slam.show_keylines)
            {
                for (const auto& line: system.lines3d | std::views::values)
                {
                    if (!_line_indices.contains(line.index))
                    {
                        _merger.addWidget(cv::viz::WLine(line[0], line[1], cv::viz::Color::green()));
                        _line_indices.insert(line.index);
                    }
                }

                _viewer->showWidget("merger", _merger);
            }

            _viewer->spinOnce(0);
        }
    }

    ImGui::Text("Hello Metal!");

    // Visualization controls with padding/grouping
    ImGui::Separator();
    ImGui::Text("Visualization Options");
    ImGui::Spacing();
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(8.0f, 6.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(10.0f, 6.0f));
    ImGui::BeginChild("viz_section", ImVec2(0, 160.0f), true);

    // Checkboxes for toggling overlays
    ImGui::Checkbox("Show Keypoints", &_options.slam.show_keypoints);
    ImGui::Checkbox("Show Keylines", &_options.slam.show_keylines);

    // Color picker for keylines (single keyline color)
    {
        const auto& s = _options.slam.keyline_single_color; // B, G, R
        ImVec4      color_rgba
        (
            static_cast<float>(s[2]) / 255.0f,
            // R
            static_cast<float>(s[1]) / 255.0f,
            // G
            static_cast<float>(s[0]) / 255.0f,
            // B
            1.0f
        );

        if (ImGui::ColorEdit3("Keyline Color", reinterpret_cast<float*>(&color_rgba)))
        {
            const auto r                       = static_cast<int>(std::round(color_rgba.x * 255.0f));
            const auto g                       = static_cast<int>(std::round(color_rgba.y * 255.0f));
            const auto b                       = static_cast<int>(std::round(color_rgba.z * 255.0f));
            _options.slam.keyline_single_color = cv::Scalar(b, g, r);
        }
    }

    ImGui::EndChild();
    ImGui::PopStyleVar(2);

    cv::waitKey(1);
}
