#include "slam_thread.h"

#include <utility>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/video/tracking.hpp>

#include <spdlog/spdlog.h>

#include <vtk-9.3/vtkLogger.h>

#include "calibration.h"
#include "grid_detector.h"
#include "utils.h"

namespace
{
    
} // namespace

zenslam::slam_thread::slam_thread(options options) : _options{ std::move(options) }
{
    vtkLogger::SetStderrVerbosity(vtkLogger::VERBOSITY_OFF);
}

zenslam::slam_thread::~slam_thread()
{
    _stop_source.request_stop();
}

void zenslam::slam_thread::track_mono(const mono_frame &frame_0, mono_frame &frame_1)
{
    // track points from the previous frame to this frame using the KLT tracker
    // KLT tracking of keypoints from previous frame to current frame (left image)
    std::vector<cv::Point2f> points_1{};
    std::vector<uchar>       status{};
    std::vector<float>       err{};

    // Convert previous keypoints to Point
    const auto &keypoints_0 = utils::values(frame_0.keypoints_);
    const auto &points_0    = utils::to_points(frame_0.keypoints_);

    if (keypoints_0.empty())
    {
        return;
    }

    cv::calcOpticalFlowPyrLK(
            frame_0.undistorted,
            frame_1.undistorted,
            points_0,
            points_1,
            status,
            err,
            cv::Size(21, 21),
            3,
            cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01),
            cv::OPTFLOW_LK_GET_MIN_EIGENVALS);

    // Verify KLT tracking results have consistent sizes
    assert(points_0.size() == points_1.size() && points_1.size() == status.size() && status.size() == err.size());

    // Update frame.l.keypoints with tracked points
    for (size_t i = 0; i < points_1.size(); ++i)
    {
        if (status[i] && err[i] < 0.1 && std::abs(points_0[i].x - points_1[i].x) < 32 &&
            std::abs(points_0[i].y - points_1[i].y) < 32)
        {
            frame_1.keypoints_.emplace(keypoints_0[i].index, keypoints_0[i]);
            frame_1.keypoints_[keypoints_0[i].index].pt = points_1[i];
        }
    }
}

void zenslam::slam_thread::correspondences_x(
        const stereo_frame            &frame,
        const std::map<size_t, point> &points,
        std::vector<cv::Point3d>      &points3d,
        std::vector<cv::Point2d>      &points2d)
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

void zenslam::slam_thread::solve_pnp(
        const cv::Matx33d              &camera_matrix,
        const std::vector<cv::Point3d> &points3d,
        const std::vector<cv::Point2d> &points2d,
        cv::Affine3d                   &pose)
{
    cv::Mat          rvec{ pose.rvec() };
    cv::Mat          tvec{ pose.translation() };
    std::vector<int> inliers{};

    SPDLOG_INFO("SolvePnP with {} points", points3d.size());

    if (cv::solvePnPRansac(
                points3d,
                points2d,
                camera_matrix,
                cv::Mat(),
                rvec,
                tvec,
                true,
                100,
                1.0,
                0.99,
                inliers,
                cv::SOLVEPNP_ITERATIVE))
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
    const auto &detector          = grid_detector(feature_detector, feature_describer, _options.slam.cell_size);
    const auto &matcher           = cv::BFMatcher::create(cv::NORM_L2, true);

    const auto &calibrations = std::vector{ calibration::parse(_options.folder.calibration_file, "cam0"),
                                            calibration::parse(_options.folder.calibration_file, "cam1") };

    SPDLOG_INFO("");
    calibrations[0].print();
    SPDLOG_INFO("");
    calibrations[1].print();

    const auto &camera_matrix_L = calibrations[0].camera_matrix();
    const auto &fundamental     = calibrations[0].fundamental(calibrations[1]);
    const auto &projection_L    = calibrations[0].projection();
    const auto &projection_R    = calibrations[1].projection();

    stereo_frame            frame_0{};
    std::map<size_t, point> points{};

    for (auto frame_1: stereo_reader)
    {
        frame_1.l.undistorted = utils::undistort(frame_1.l.image, calibrations[0]);
        frame_1.r.undistorted = utils::undistort(frame_1.r.image, calibrations[1]);

        // track keypoints new
        track_mono(frame_0.l, frame_1.l);
        track_mono(frame_0.r, frame_1.r);

        SPDLOG_INFO("KLT tracked {} keypoints from previous frame in L", frame_1.l.keypoints_.size());
        SPDLOG_INFO("KLT tracked {} keypoints from previous frame in R", frame_1.r.keypoints_.size());

        // detect new
        detector.detect(frame_1.l.undistorted, frame_1.l.keypoints_);
        detector.detect(frame_1.r.undistorted, frame_1.r.keypoints_);

        SPDLOG_INFO("Detected points L: {}", frame_1.l.keypoints_.size());
        SPDLOG_INFO("Detected points R: {}", frame_1.r.keypoints_.size());

        // match keypoints
        utils::match(frame_1.l.keypoints_, frame_1.r.keypoints_, fundamental, _options.slam.epipolar_threshold);

        // Before we compute 3D-2D pose we should compute the 3D-3D pose
        utils::triangulate(frame_1, projection_L, projection_R, frame_1.points);
        SPDLOG_INFO("3D points count: {}", frame_1.points.size());

        // Gather points from frame_0 and frame_1 for 3D-3D pose computation
        std::vector<cv::Point3d> points3d_0{};
        std::vector<cv::Point3d> points3d_1{};
        for (const auto &[index, point]: frame_0.points)
        {
            if (frame_1.points.contains(index) && points.contains(index))
            {
                points3d_0.emplace_back(frame_0.points.at(index));
                points3d_1.emplace_back(frame_1.points.at(index));
            }
        }

        // Compute relative pose between frame_0 and frame_1 using 3D-3D correspondences
        if (points3d_0.size() >= 6)
        {
            cv::Matx33d R;
            cv::Vec3d   t;
            utils::umeyama(points3d_0, points3d_1, R, t);
            frame_1.pose = frame_0.pose * cv::Affine3d(R, t).inv();
            SPDLOG_INFO("Pose: {}", frame_1.pose);
        }

        // compute PnP pose
        std::vector<cv::Point3d> points3d;
        std::vector<cv::Point2d> points2d;
        correspondences_x(frame_1, points, points3d, points2d);

        if (points3d.size() >= 6)
        {
            auto pose_of_world_in_camera = frame_0.pose.inv();

            try
            {
                solve_pnp(camera_matrix_L, points3d, points2d, pose_of_world_in_camera);
            }
            catch (std::exception &e)
            {
                SPDLOG_WARN("SolvePnP failed: {}", e.what());
                break;
            }

            frame_1.pose = pose_of_world_in_camera.inv();
        }
        else
        {
            SPDLOG_WARN("Not enough points for PnP: {}", points3d.size());
        }

        // triangulate filtered matches to get 3D points
        utils::triangulate(frame_1, calibrations[0].projection(frame_1.pose), calibrations[1].projection(frame_1.pose), points);
        SPDLOG_INFO("3D points count: {}", points.size());

        frame_1.points3d = points | std::views::values | std::views::transform([](const auto &p) { return cv::Point3d(p); }) |
                           std::ranges::to<std::vector>();

        on_frame(frame_1);

        frame_0 = std::move(frame_1);

        if (_stop_token.stop_requested())
        {
            break;
        }
    }
}
