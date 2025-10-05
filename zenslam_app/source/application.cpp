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
    // display matches spatial
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
    // if (!_frame_1->points.empty())
    // {
    //     if (!_viewer)
    //     {
    //         _viewer = std::make_unique<cv::viz::Viz3d>("3D Points");
    //         _viewer->setWindowSize(cv::Size(800, 600));
    //         _viewer->setWindowPosition(cv::Point(700, 100));
    //         _viewer->setBackgroundColor(); // black by default
    //         _viewer->showWidget("Coordinate Widget", cv::viz::WCoordinateSystem(0.1));
    //     }
    //     else
    //     {
    //         _viewer->removeAllWidgets();
    //         _viewer->showWidget("Coordinate Widget", cv::viz::WCoordinateSystem(0.1));
    //
    //         auto cloud = cv::viz::WCloud
    //         (
    //             _frame_1->points,
    //             cv::viz::Color::green()
    //         );
    //
    //         cloud.setRenderingProperty(cv::viz::POINT_SIZE, 4.0);
    //
    //         _viewer->showWidget("Point Cloud", cloud);
    //         _viewer->spinOnce(0, true);
    //     }
    // }

    cv::waitKey(1);
}
