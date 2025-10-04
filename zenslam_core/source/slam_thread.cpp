#include "slam_thread.h"

#include <opencv2/core/types.hpp>
#include <queue>
#include <utility>

#include <opencv2/calib3d.hpp>

#include <spdlog/spdlog.h>

#include "calibration.h"
#include "grid_detector.h"
#include "stereo_folder_reader.h"
#include "utils.h"

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
    const auto &feature_detector  = cv::FastFeatureDetector::create(8);
    const auto &feature_describer = cv::SiftDescriptorExtractor::create();
    const auto &detector          = grid_detector::create(feature_detector, _options.slam.cell_size);
    const auto &matcher           = cv::BFMatcher::create(cv::NORM_L2, true);

    auto calibrations = std::vector
    {
        calibration::parse(_options.folder.calibration_file, "cam0"),
        calibration::parse(_options.folder.calibration_file, "cam1")
    };

    SPDLOG_INFO("");
    calibrations[0].print();
    SPDLOG_INFO("");
    calibrations[1].print();

    const auto &fundamental = calibrations[0].fundamental(calibrations[1]);
    const auto &projection0 = calibrations[0].projection();
    const auto &projection1 = calibrations[1].projection();

    auto queue = std::queue<stereo_frame> { };

    for (auto frame: stereo_reader)
    {
        frame.l.undistorted = utils::undistort(frame.l.image, calibrations[0]);
        frame.r.undistorted = utils::undistort(frame.r.image, calibrations[1]);

        detector->detect(frame.l.undistorted, frame.l.keypoints, cv::noArray());
        detector->detect(frame.r.undistorted, frame.r.keypoints, cv::noArray());

        SPDLOG_INFO("Detected points L: {}", frame.l.keypoints.size());
        SPDLOG_INFO("Detected points R: {}", frame.r.keypoints.size());

        cv::Mat descriptors_l;
        cv::Mat descriptors_r;

        feature_describer->compute(frame.l.undistorted, frame.l.keypoints, descriptors_l);
        feature_describer->compute(frame.r.undistorted, frame.r.keypoints, descriptors_r);

        std::vector<cv::DMatch> matches;
        matcher->match(descriptors_l, descriptors_r, matches);
        frame.matches = matches;

        SPDLOG_INFO("Matches before epipolar filtering: {}", frame.matches.size());

        std::tie(frame.filtered, frame.unmatched) = utils::filter
        (
            frame.l.keypoints,
            frame.r.keypoints,
            matches,
            fundamental,
            _options.slam.epipolar_threshold
        );

        SPDLOG_INFO("Matches after epipolar filtering: {}", frame.filtered.size());

        // triangulate filtered matches to get 3D points
        if (!frame.filtered.empty())
        {
            frame.points = utils::triangulate
            (
                frame.l.keypoints,
                frame.r.keypoints,
                frame.filtered,
                projection0,
                projection1
            );

            SPDLOG_INFO("Triangulated 3D points: {}", frame.points.size());
        }

        on_frame(frame);

        if (_stop_token.stop_requested())
        {
            break;
        }
    }
}
