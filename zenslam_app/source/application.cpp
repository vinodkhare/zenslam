#include "application.h"

#include <opencv2/highgui.hpp>
#include <opencv2/viz.hpp>

#include <opencv2/viz/widgets.hpp>
#include <zenslam/utils.h>

#include <numbers>

zenslam::application::application(const options &options) :
    _options { options }
{
    _slam_thread.on_frame += [this](const stereo_frame &frame)
    {
        _frame_0 = _frame_1;
        _frame_1 = frame;
    };
}

void zenslam::application::render()
{
    // display matches spatial
    if (_frame_1->l.keypoints.empty()) return;

    {
        const auto &matches_image = utils::draw_matches(_frame_1);

        cv::imshow("matches_spatial", matches_image);
        cv::setWindowTitle("matches_spatial", "matches spatial");
    }

    // display matches temporal
    {
        const auto &matches_image = utils::draw_matches(_frame_0->l, _frame_1->l);

        cv::imshow("matches_temporal", matches_image);
        cv::setWindowTitle("matches_temporal", "matches temporal");
    }

    // display 3D points using viz module
    {
        if (!_viewer)
        {
            _viewer = std::make_unique<cv::viz::Viz3d>("3D Points");
            _viewer->setGlobalWarnings(false);

            _viewer->setWindowSize(cv::Size(800, 600));
            _viewer->setWindowPosition(cv::Point(700, 100));
            _viewer->setBackgroundColor(); // black by default
            _viewer->showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
        }
        else
        {
            _viewer->removeAllWidgets();
            _viewer->showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
            _viewer->showWidget("camera", cv::viz::WCameraPosition(cv::Vec2d { std::numbers::pi / 2, std::numbers::pi / 2 }, _frame_1->l.undistorted));
            _viewer->setWidgetPose("camera", _frame_1->pose);

            _viewer->showWidget("cloud", cv::viz::WCloud(_frame_1->points3d));
            _viewer->setRenderingProperty("cloud", cv::viz::POINT_SIZE, 4.0);

            _viewer->spinOnce(0, true);

        }
    }

    cv::waitKey(1);
}
