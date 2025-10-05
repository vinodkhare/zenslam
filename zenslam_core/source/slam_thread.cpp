#include "slam_thread.h"

#include <queue>
#include <utility>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/video/tracking.hpp>

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

void zenslam::slam_thread::track
(
    const stereo_frame &frame_0,
    stereo_frame &      frame_1
)
{
    // track points from previous frame to this frame using the KLT tracker
    // KLT tracking of keypoints from previous frame to current frame (left image)
    std::vector<cv::Point2f> points_0 { };
    std::vector<cv::Point2f> points_1 { };
    std::vector<uchar>       status { };
    std::vector<float>       err { };

    // Convert previous keypoints to Point2f
    points_0 = utils::to_points(frame_0.l.keypoints);

    cv::calcOpticalFlowPyrLK
    (
        frame_0.l.undistorted,
        frame_1.l.undistorted,
        points_0,
        points_1,
        status,
        err
    );

    // Verify KLT tracking results have consistent sizes
    assert(points_0.size() == points_1.size() && points_1.size() == status.size() && status.size() == err.size());

    // Update frame.l.keypoints with tracked points
    size_t j = 0;
    for (size_t i = 0; i < points_1.size(); ++i)
    {
        if (status[i])
        {
            frame_1.l.keypoints.emplace_back(frame_0.l.keypoints[i]);
            frame_1.l.keypoints.back().pt = points_1[i];

            frame_1.temporal.matches.emplace_back(i, j++, err[i]);
        }
    }
}

void zenslam::slam_thread::correspondences
(
    const stereo_frame &      frame_0,
    const stereo_frame &      frame_1,
    std::vector<cv::Point3d> &points3d,
    std::vector<cv::Point2d> &points2d
)
{
    const auto &matches_map = utils::to_map(frame_0.spatial.filtered);

    for (const auto &match: frame_1.temporal.matches)
    {
        if (matches_map.contains(match.queryIdx))
        {
            points3d.emplace_back
            (
                frame_0.points[std::distance(matches_map.begin(), matches_map.find(match.queryIdx))]
            );

            points2d.emplace_back(frame_1.l.keypoints[match.trainIdx].pt);
        }
    }
}

void zenslam::slam_thread::loop()
{
    const auto &stereo_reader = stereo_folder_reader(_options.folder);

    // Create a base detector (FAST)
    const auto &feature_detector  = cv::FastFeatureDetector::create(8);
    const auto &feature_describer = cv::SiftDescriptorExtractor::create();
    const auto &detector          = grid_detector::create(feature_detector, _options.slam.cell_size);
    const auto &matcher           = cv::BFMatcher::create(cv::NORM_L2, true);

    auto calibrations = std::vector {
        calibration::parse(_options.folder.calibration_file, "cam0"),
        calibration::parse(_options.folder.calibration_file, "cam1")
    };

    SPDLOG_INFO("");
    calibrations[0].print();
    SPDLOG_INFO("");
    calibrations[1].print();

    const auto &camera_matrix_L = calibrations[0].camera_matrix();
    const auto &fundamental     = calibrations[0].fundamental(calibrations[1]);
    const auto &projection_L    = calibrations[0].projection();
    const auto &projection_R    = calibrations[1].projection();

    std::optional<stereo_frame> frame_0 { };

    for (auto frame_1: stereo_reader)
    {
        frame_1.l.undistorted = utils::undistort(frame_1.l.image, calibrations[0]);
        frame_1.r.undistorted = utils::undistort(frame_1.r.image, calibrations[1]);

        if (frame_0.has_value())
        {
            // track keypoints from previous frame
            track(frame_0.value(), frame_1);

            SPDLOG_INFO("KLT tracked {} keypoints from previous frame", frame_1.l.keypoints.size());

            // detect more keypoints in empty cells
            detector->detect(frame_1.l.undistorted, frame_1.l.keypoints);

            // compute PnP pose
            std::vector<cv::Point3d> points3d;
            std::vector<cv::Point2d> points2d;
            correspondences(frame_0.value(), frame_1, points3d, points2d);

            if (points3d.size() >= 6)
            {
                cv::Mat          rvec { };
                cv::Mat          tvec { };
                std::vector<int> inliers { };

                if
                (
                    cv::solvePnPRansac
                    (
                        points3d,
                        points2d,
                        camera_matrix_L,
                        cv::Mat(),
                        rvec,
                        tvec,
                        false,
                        100,
                        8.0,
                        0.99,
                        inliers,
                        cv::SOLVEPNP_ITERATIVE
                    )
                )
                {
                    SPDLOG_INFO("PnP successful with {} inliers out of {} points", inliers.size(), points3d.size());

                    auto pose = cv::Affine3d(rvec, tvec);
                    SPDLOG_INFO("Pose: {}", pose);
                }
                else
                {
                    SPDLOG_WARN("PnP failed");
                }
            }
            else
            {
                SPDLOG_WARN("Not enough points for PnP: {}", points3d.size());
            }
        }
        else
        {
            detector->detect(frame_1.l.undistorted, frame_1.l.keypoints);
            detector->detect(frame_1.r.undistorted, frame_1.r.keypoints);

            SPDLOG_INFO("Detected points L: {}", frame_1.l.keypoints.size());
            SPDLOG_INFO("Detected points R: {}", frame_1.r.keypoints.size());

            cv::Mat descriptors_l;
            cv::Mat descriptors_r;

            feature_describer->compute(frame_1.l.undistorted, frame_1.l.keypoints, descriptors_l);
            feature_describer->compute(frame_1.r.undistorted, frame_1.r.keypoints, descriptors_r);

            std::vector<cv::DMatch> matches;
            matcher->match(descriptors_l, descriptors_r, matches);
            frame_1.spatial.matches = matches;

            SPDLOG_INFO("Matches before epipolar filtering: {}", frame_1.spatial.matches.size());

            std::tie(frame_1.spatial.filtered, frame_1.spatial.unmatched) = utils::filter
            (
                frame_1.l.keypoints,
                frame_1.r.keypoints,
                matches,
                fundamental,
                _options.slam.epipolar_threshold
            );

            SPDLOG_INFO("Matches after epipolar filtering: {}", frame_1.spatial.filtered.size());

            // triangulate filtered matches to get 3D points
            if (!frame_1.spatial.filtered.empty())
            {
                frame_1.points =
                        utils::triangulate
                        (frame_1.l.keypoints, frame_1.r.keypoints, frame_1.spatial.filtered, projection_L, projection_R);

                SPDLOG_INFO("Triangulated 3D points: {}", frame_1.points.size());
            }
        }

        frame_0 = std::move(frame_1);

        on_frame(frame_0.value());

        if (_stop_token.stop_requested())
        {
            break;
        }
    }
}
