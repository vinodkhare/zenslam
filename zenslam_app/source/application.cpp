#include "application.h"

#include <opencv2/highgui.hpp>
#include <opencv2/viz.hpp>

#include <opencv2/viz/widgets.hpp>
#include <zenslam/utils.h>

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
    if (!_frame_1->l.undistorted.empty())
    {
        cv::imshow("L", _frame_1->l.undistorted);
        cv::setWindowTitle
        (
            "L",
            std::format("L: {{ t: {} }}", utils::to_string_epoch(_frame_1->l.timestamp))
        );

        if (!_frame_1->l.keypoints.empty())
        {
            const auto &keypoints_image = utils::draw_keypoints(_frame_1->l);

            cv::imshow("keypoints_L", keypoints_image);
            cv::setWindowTitle("keypoints_L", "Keypoints L");
        }
    }

    if (!_frame_1->r.undistorted.empty())
    {
        cv::imshow("R", _frame_1->r.undistorted);
        cv::setWindowTitle
        (
            "R",
            std::format("R: {{ t: {} }}", utils::to_string_epoch(_frame_1->r.timestamp))
        );

        if (!_frame_1->r.keypoints.empty())
        {
            const auto &keypoints_image = utils::draw_keypoints(_frame_1->r);

            cv::imshow("keypoints_R", keypoints_image);
            cv::setWindowTitle("keypoints_R", "Keypoints R");
        }
    }

    // display matches
    if (!_frame_1->spatial.matches.empty())
    {
        const auto &matches_image = utils::draw_matches(_frame_1);

        cv::imshow("matches_spatial", matches_image);
        cv::setWindowTitle("matches_spatial", "matches spatial");
    }

    // display matches
    if (!_frame_1->temporal.matches.empty())
    {
        const auto &matches_image = utils::draw_matches(_frame_0, _frame_1);

        cv::imshow("matches_temporal", matches_image);
        cv::setWindowTitle("matches_temporal", "matches temporal");
    }

    // display 3D points using viz module
    if (!_frame_1->points.empty())
    {
        if (!_viewer)
        {
            _viewer = std::make_unique<cv::viz::Viz3d>("3D Points");
            _viewer->setWindowSize(cv::Size(800, 600));
            _viewer->setWindowPosition(cv::Point(700, 100));
            _viewer->setBackgroundColor(); // black by default
            _viewer->showWidget("Coordinate Widget", cv::viz::WCoordinateSystem(0.1));
        }
        else
        {
            _viewer->removeAllWidgets();
            _viewer->showWidget("Coordinate Widget", cv::viz::WCoordinateSystem(0.1));

            auto cloud = cv::viz::WCloud
            (
                _frame_1->points,
                cv::viz::Color::green()
            );

            cloud.setRenderingProperty(cv::viz::POINT_SIZE, 4.0);

            _viewer->showWidget("Point Cloud", cloud);
            _viewer->spinOnce(0, true);
        }
    }

    cv::waitKey(1);
}
