#include "slam_thread.h"

#include <queue>
#include <utility>

#include <spdlog/spdlog.h>

#include "grid_detector.h"
#include "stereo_folder_reader.h"

zenslam::slam_thread::slam_thread(options options) :
    _options { std::move(options) } {}

zenslam::slam_thread::~slam_thread()
{
    _stop_source.request_stop();
}

void zenslam::slam_thread::loop()
{
    const auto &stereo_reader = stereo_folder_reader(_options.folder);

    // Create a base detector (FAST)
    const auto &feature_detector  = cv::FastFeatureDetector::create(32);
    const auto &feature_describer = cv::ORB::create();
    const auto &detector          = grid_detector::create(feature_detector, _options.slam.cell_size);
    const auto &matcher           = cv::BFMatcher::create(cv::NORM_L2, false);

    auto queue = std::queue<stereo_frame> { };

    for (auto frame: stereo_reader)
    {
        detector->detect(frame.l.image, frame.l.keypoints, cv::noArray());
        detector->detect(frame.r.image, frame.r.keypoints, cv::noArray());

        SPDLOG_INFO("Detected points L: {}", frame.l.keypoints.size());
        SPDLOG_INFO("Detected points R: {}", frame.r.keypoints.size());

        cv::Mat descriptors_l;
        cv::Mat descriptors_r;

        feature_describer->compute(frame.l.image, frame.l.keypoints, descriptors_l);
        feature_describer->compute(frame.r.image, frame.r.keypoints, descriptors_r);

        std::vector<cv::DMatch> matches = { };
        matcher->match(descriptors_l, descriptors_r, matches);
        SPDLOG_INFO("Matches: {}", matches.size());

        frame.matches = matches;

        on_frame(frame);

        if (_stop_token.stop_requested())
        {
            break;
        }
    }
}
