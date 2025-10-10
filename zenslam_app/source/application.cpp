#include "application.h"

#include <opencv2/highgui.hpp>
#include <opencv2/viz.hpp>

#include <opencv2/viz/widgets.hpp>
#include <zenslam/utils.h>

#include <numbers>
#include <utility>

zenslam::application::application(options options) :
    _options {std::move( options )}
{
    _slam_thread.on_frame += [this](const stereo_frame &frame)
    {
        std::lock_guard lock { _mutex };

        _frame_0 = _frame_1;
        _frame_1 = frame;
    };
}

void zenslam::application::render()
{
    stereo_frame frame_0 { };
    stereo_frame frame_1 { };
    {
        std::lock_guard lock { _mutex };

        frame_0 = _frame_0;
        frame_1 = _frame_1;
    }

    // display matches spatial
    if (frame_1.l.keypoints.empty() || frame_0.l.undistorted.empty() || frame_1.l.undistorted.empty()) return;

    {
        const auto &matches_image = utils::draw_matches(frame_1);

        cv::imshow("matches_spatial", matches_image);
        cv::setWindowTitle("matches_spatial", "matches spatial");
        cv::resizeWindow("matches_spatial", 1024, 512);
    }

    // display matches temporal
    {
        const auto &matches_image = utils::draw_matches(frame_0.l, frame_1.l);

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
        else
        {
            _viewer->removeAllWidgets();

            _viewer->showWidget("origin", cv::viz::WCoordinateSystem());

            _viewer->showWidget("camera", cv::viz::WCameraPosition(cv::Vec2d { std::numbers::pi / 2, std::numbers::pi / 2 }, frame_1.l.undistorted));
            _viewer->setWidgetPose("camera", frame_1.pose);

            _viewer->showWidget("cloud", cv::viz::WCloud(frame_1.points3d, frame_1.colors));
            _viewer->setRenderingProperty("cloud", cv::viz::POINT_SIZE, 4.0);

            _viewer->spinOnce(0, true);
        }
    }

    cv::waitKey(1);
}
