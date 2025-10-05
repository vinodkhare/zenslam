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

void zenslam::slam_thread::track_mono(const mono_frame &frame_0, mono_frame &frame_1)
{
    // track points from the previous frame to this frame using the KLT tracker
    // KLT tracking of keypoints from previous frame to current frame (left image)
    std::vector<cv::Point2f> points_1 { };
    std::vector<uchar>       status { };
    std::vector<float>       err { };

    // Convert previous keypoints to Point
    const auto &keypoints_0 = utils::values(frame_0.keypoints_);
    const auto &points_0    = utils::to_points(frame_0.keypoints_);

    cv::calcOpticalFlowPyrLK
    (
        frame_0.undistorted,
        frame_1.undistorted,
        points_0,
        points_1,
        status,
        err
    );

    // Verify KLT tracking results have consistent sizes
    assert(points_0.size() == points_1.size() && points_1.size() == status.size() && status.size() == err.size());

    // Update frame.l.keypoints with tracked points
    for (size_t i = 0; i < points_1.size(); ++i)
    {
        if (status[i])
        {
            frame_1.keypoints_.emplace(keypoints_0[i].index, keypoints_0[i]);
            frame_1.keypoints_[keypoints_0[i].index].pt = points_1[i];
        }
    }
}

void zenslam::slam_thread::correspondences_x
(
    const stereo_frame &           frame,
    const std::map<size_t, point> &points,
    std::vector<cv::Point3d> &     points3d,
    std::vector<cv::Point2d> &     points2d
)
{
    for (const auto &[index, keypoint_l]: frame.l.keypoints_)
    {
        if (points.contains(index))
        {
            points3d.emplace_back(points.at(index));
            points2d.emplace_back(keypoint_l.pt);
        }
    }
}

void zenslam::slam_thread::solve_pnp
(
    const cv::Matx33d &             camera_matrix,
    const std::vector<cv::Point3d> &points3d,
    const std::vector<cv::Point2d> &points2d,
    cv::Affine3d &                  pose
)
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
            camera_matrix,
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
        SPDLOG_INFO("SolvePnP successful with {} inliers out of {} points", inliers.size(), points3d.size());

        pose = cv::Affine3d(rvec, tvec);
        SPDLOG_INFO("Pose: {}", pose);
    }
    else
    {
        SPDLOG_WARN("SolvePnP failed");
        throw std::runtime_error("SolvePnP failed");
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

    const auto &calibrations = std::vector
    {
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

    std::optional<stereo_frame> frame_container { };
    std::map<size_t, point>     points { };

    for (auto frame_1: stereo_reader)
    {
        frame_1.l.undistorted = utils::undistort(frame_1.l.image, calibrations[0]);
        frame_1.r.undistorted = utils::undistort(frame_1.r.image, calibrations[1]);

        if (frame_container.has_value())
        {
            const auto &frame_0 = frame_container.value();

            // track keypoints new
            track_mono(frame_0.l, frame_1.l);
            SPDLOG_INFO("KLT tracked {} keypoints from previous frame", frame_1.l.keypoints_.size());

            // detect new
            detector->detect(frame_1.l.undistorted, frame_1.l.keypoints_);
            SPDLOG_INFO("Detected additional keypoints now total {}", frame_1.l.keypoints_.size());

            // compute PnP pose
            std::vector<cv::Point3d> points3d;
            std::vector<cv::Point2d> points2d;
            correspondences_x(frame_1, points, points3d, points2d);

            if (points3d.size() >= 6)
            {
                cv::Affine3d pose;

                try
                {
                    solve_pnp(camera_matrix_L, points3d, points2d, pose);
                }
                catch (std::exception &e)
                {
                    SPDLOG_WARN("SolvePnP failed: {}", e.what());
                    break;
                }

                frame_1.pose = pose * frame_0.pose;
            }
            else
            {
                SPDLOG_WARN("Not enough points for PnP: {}", points3d.size());
            }
        }
        else
        {
            detector->detect(frame_1.l.undistorted, frame_1.l.keypoints_);
            detector->detect(frame_1.r.undistorted, frame_1.r.keypoints_);

            SPDLOG_INFO("Detected points L: {}", frame_1.l.keypoints_.size());
            SPDLOG_INFO("Detected points R: {}", frame_1.r.keypoints_.size());

            cv::Mat descriptors_l;
            cv::Mat descriptors_r;

            auto keypoints_l = utils::cast<cv::KeyPoint>(utils::values(frame_1.l.keypoints_));
            auto keypoints_r = utils::cast<cv::KeyPoint>(utils::values(frame_1.r.keypoints_));

            feature_describer->compute(frame_1.l.undistorted, keypoints_l, descriptors_l);
            feature_describer->compute(frame_1.r.undistorted, keypoints_r, descriptors_r);

            std::vector<cv::DMatch> matches;
            matcher->match(descriptors_l, descriptors_r, matches);
            frame_1.spatial.matches = matches;

            SPDLOG_INFO("Matches before epipolar filtering: {}", frame_1.spatial.matches.size());

            std::tie(frame_1.spatial.filtered, frame_1.spatial.unmatched) = utils::filter
            (
                keypoints_l,
                keypoints_r,
                matches,
                fundamental,
                _options.slam.epipolar_threshold
            );

            SPDLOG_INFO("Matches after epipolar filtering: {}", frame_1.spatial.filtered.size());

            const auto &keypoints_L = frame_1.l.keypoints_ | std::ranges::views::values | std::ranges::to<std::vector>();
            const auto &keypoints_R = frame_1.r.keypoints_ | std::ranges::views::values | std::ranges::to<std::vector>();

            for (const auto &match: frame_1.spatial.filtered)
            {
                const auto &keypoint_L = keypoints_L[match.queryIdx];
                auto        keypoint_R = keypoints_R[match.trainIdx];

                frame_1.r.keypoints_.erase(keypoint_R.index);

                keypoint_R.index = keypoint_L.index;

                frame_1.r.keypoints_.emplace(keypoint_R.index, keypoint_R);
            }

            // triangulate filtered matches to get 3D points
            if (!frame_1.spatial.filtered.empty())
            {
                utils::triangulate(frame_1, projection_L, projection_R, points);

                SPDLOG_INFO("Triangulated 3D points: {}", points.size());
            }
        }

        on_frame(frame_1);

        frame_container = std::move(frame_1);

        if (_stop_token.stop_requested())
        {
            break;
        }
    }
}
