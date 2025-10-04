#include "slam_thread.h"

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

    auto fundamental = calibrations[0].fundamental(calibrations[1]);

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
        SPDLOG_INFO("Matches before epipolar filtering: {}", matches.size());

        // Prepare points for filtering
        std::vector<cv::Point2f> pts_l, pts_r;
        for (const auto &m: matches)
        {
            pts_l.push_back(frame.l.keypoints[m.queryIdx].pt);
            pts_r.push_back(frame.r.keypoints[m.trainIdx].pt);
        }

        // Compute epipolar error for each match and filter
        std::vector<cv::DMatch> filtered_matches;
        constexpr auto          epipolar_threshold = 1.0; // pixels, adjust as needed

        if (pts_l.size() == pts_r.size() && !pts_l.empty())
        {
            std::vector<cv::Vec3f> epilines_l, epilines_r;
            cv::computeCorrespondEpilines(pts_l, 1, fundamental, epilines_r); // lines in right image
            cv::computeCorrespondEpilines(pts_r, 2, fundamental, epilines_l); // lines in left image

            for (size_t i = 0; i < matches.size(); ++i)
            {
                const auto &pt_l   = pts_l[i];
                const auto &pt_r   = pts_r[i];
                const auto &line_r = epilines_r[i];
                const auto &line_l = epilines_l[i];

                // Distance from right point to left epipolar line
                double err_l = std::abs(line_l[0] * pt_l.x + line_l[1] * pt_l.y + line_l[2]) /
                               std::sqrt(line_l[0] * line_l[0] + line_l[1] * line_l[1]);

                // Distance from left point to right epipolar line
                double err_r = std::abs(line_r[0] * pt_r.x + line_r[1] * pt_r.y + line_r[2]) /
                               std::sqrt(line_r[0] * line_r[0] + line_r[1] * line_r[1]);

                if (err_l < epipolar_threshold && err_r < epipolar_threshold)
                {
                    filtered_matches.push_back(matches[i]);
                }
            }
        }
        else
        {
            filtered_matches = matches; // fallback: no filtering
        }
        SPDLOG_INFO("Matches after epipolar filtering: {}", filtered_matches.size());

        frame.matches = filtered_matches;

        on_frame(frame);

        if (_stop_token.stop_requested())
        {
            break;
        }
    }
}
