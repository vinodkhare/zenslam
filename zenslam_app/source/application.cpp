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
    _options { std::move(options) }
{
    _slam_thread.on_frame += [this](const zenslam::frame::slam &slam)
    {
        std::lock_guard lock { _mutex };

        _slam = slam;
    };
}

void zenslam::application::render()
{
    frame::slam slam { };
    {
        std::lock_guard lock { _mutex };
        slam = _slam;
    }

    // display matches spatial
    if (slam.frames[1].cameras[0].keypoints.empty() || slam.frames[0].cameras[0].undistorted.empty() || slam.frames[1].cameras[
            0].undistorted.empty())
        return;

    {
        const auto &matches_image = utils::draw_matches(slam.frames[1], slam.points3d_map);

        cv::imshow("matches_spatial", matches_image);
        cv::setWindowTitle("matches_spatial", "matches spatial");
        cv::resizeWindow("matches_spatial", 1024, 512);
    }

    // display matches temporal
    {
        const auto &matches_image = utils::draw_matches(slam.frames[0].cameras[0], slam.frames[1].cameras[0]);

        cv::namedWindow("matches_temporal");
        cv::imshow("matches_temporal", matches_image);
        cv::setWindowTitle("matches_temporal", "matches temporal");
        cv::resizeWindow("matches_spatial", 1024, 512);
    }

    // display 3D points using viz module
    {
        if (!_viewer)
        {
            _viewer = std::make_unique<cv::viz::Viz3d>("3D Points");

            _viewer->setBackgroundColor(cv::viz::Color::white());
            _viewer->setWindowPosition(cv::Point(700, 100));
            _viewer->setWindowSize(cv::Size(1024, 1024));
        }
        else if (!slam.points3d_map.empty())
        {
            _viewer->removeAllWidgets();

            _viewer->showWidget("origin", cv::viz::WCoordinateSystem());

            cv::viz::WCameraPosition camera_position
            (
                cv::Vec2d { std::numbers::pi / 2, std::numbers::pi / 2 },
                slam.frames[1].cameras[0].undistorted
            );

            camera_position.setColor(cv::viz::Color::red());
            _viewer->showWidget("camera", camera_position);
            _viewer->setWidgetPose("camera", slam.frames[1].pose);

            cv::viz::WCameraPosition camera_gt
            (
                cv::Vec2d { std::numbers::pi / 2, std::numbers::pi / 2 },
                slam.frames[1].cameras[0].undistorted
            );

            camera_gt.setColor(cv::viz::Color::bluberry());
            _viewer->showWidget("camera_gt", camera_gt);
            _viewer->setWidgetPose("camera_gt", slam.frames[1].pose_gt);

            const auto &points = slam.points3d_map | std::views::values | std::views::transform
                                 (
                                     [](const auto &p)
                                     {
                                         return cv::Point3d { p.x, p.y, p.z };
                                     }
                                 ) | std::ranges::to<std::vector>();

            const auto &colors = slam.points3d_map | std::views::values | std::views::transform
                                 (
                                     [](const auto &p)
                                     {
                                         return p.color;
                                     }
                                 ) | std::ranges::to<std::vector>();

            _viewer->showWidget("cloud", cv::viz::WCloud(points, colors));
            _viewer->setRenderingProperty("cloud", cv::viz::POINT_SIZE, 4.0);

            for (const auto &[index, line]: slam.lines3d_map)
            {
                _viewer->showWidget
                (
                    "line_" + std::to_string(index),
                    cv::viz::WLine(line.points3d[0], line.points3d[1], cv::viz::Color::green())
                );
            }

            _viewer->spinOnce(0, true);
        }
    }

    ImGui::Text("Hello Metal!");

    cv::waitKey(1);
}
