#include "application.h"

#include <opencv2/highgui.hpp>

#include "utils.h"

zenslam::application::application(const options &options) :
    _options { options }
{
    _slam_thread.on_frame += [this](const stereo_frame &frame)
    {
        _frame = frame;
    };
}

void zenslam::application::render()
{
    if (!_frame->l.image.empty())
    {
        cv::imshow("L", _frame->l.image);
        cv::setWindowTitle
        (
            "L",
            std::format("L: {{ t: {} }}", utils::epoch_double_to_string(_frame->l.timestamp))
        );

        if (!_frame->l.keypoints.empty())
        {
            const auto &keypoints_image = utils::draw_keypoints(_frame->l);

            cv::imshow("keypoints_L", keypoints_image);
            cv::setWindowTitle("keypoints_L", "Keypoints L");
        }
    }

    if (!_frame->r.image.empty())
    {
        cv::imshow("R", _frame->r.image);
        cv::setWindowTitle
        (
            "R",
            std::format("R: {{ t: {} }}", utils::epoch_double_to_string(_frame->r.timestamp))
        );

        if (!_frame->r.keypoints.empty())
        {
            const auto &keypoints_image = utils::draw_keypoints(_frame->r);

            cv::imshow("keypoints_R", keypoints_image);
            cv::setWindowTitle("keypoints_R", "Keypoints R");
        }
    }

    // display matches
    if (!_frame->matches.empty())
    {
        const auto &matches_image = utils::draw_matches(_frame);

        cv::imshow("matches", matches_image);
        cv::setWindowTitle("matches", "matches");
    }

    cv::waitKey(1);
}
