#include "slam_thread.h"


#include <utility>

#include <gsl/narrow>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/types.hpp>

#include <spdlog/spdlog.h>

#include <vtk-9.3/vtkLogger.h>

#include "calibration.h"
#include "grid_detector.h"
#include "groundtruth.h"
#include "motion.h"
#include "slam_frame.h"
#include "time_this.h"
#include "utils.h"
#include "utils_opencv.h"
#include "utils_slam.h"

zenslam::slam_thread::slam_thread(options options) :
    _options { std::move(options) }
{
    vtkLogger::SetStderrVerbosity(vtkLogger::VERBOSITY_OFF);
}


zenslam::slam_thread::~slam_thread()
{
    _stop_source.request_stop();
}


void zenslam::slam_thread::loop()
{
    const auto &stereo_reader     = stereo_folder_reader(_options.folder);
    const auto &feature_detector  = cv::FastFeatureDetector::create(8);
    const auto &feature_describer = cv::SiftDescriptorExtractor::create();
    const auto &detector          = grid_detector(feature_detector, feature_describer, _options.slam.cell_size);
    const auto &clahe             = cv::createCLAHE();

    auto groundtruth = groundtruth::read(_options.folder.groundtruth_file);
    auto motion      = zenslam::motion();

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

    slam_frame slam { };

    for (auto f: stereo_reader)
    {
        slam.frame[0] = std::move(slam.frame[1]);
        slam.frame[1] = std::move(f);

        const auto &dt    = isnan(slam.frame[0].l.timestamp) ? 0.0 : slam.frame[1].l.timestamp - slam.frame[0].l.timestamp;
        const auto &slerp = groundtruth.slerp(slam.frame[1].l.timestamp);

        slam.frame[1].pose_gt = cv::Affine3d { slerp.quaternion.toRotMat3x3(), slerp.translation };
        if (isnan(slam.frame[0].l.timestamp)) slam.frame[0].pose = slam.frame[1].pose_gt;
        slam.frame[1].pose = motion.predict(slam.frame[0].pose, dt);

        SPDLOG_DEBUG("Predicted pose: {}", slam.frame[1].pose);

        cv::cvtColor(slam.frame[1].l.image, slam.frame[1].l.image, cv::COLOR_BGR2GRAY);
        cv::cvtColor(slam.frame[1].r.image, slam.frame[1].r.image, cv::COLOR_BGR2GRAY);

        if (_options.slam.clahe_enabled)
        {
            clahe->apply(slam.frame[1].l.image, slam.frame[1].l.image);
            clahe->apply(slam.frame[1].r.image, slam.frame[1].r.image);
        }

        slam.frame[1].l.undistorted = utils::undistort(slam.frame[1].l.image, calibrations[0]);
        slam.frame[1].r.undistorted = utils::undistort(slam.frame[1].r.image, calibrations[1]);

        slam.frame[1].l.pyramid = utils::pyramid(slam.frame[1].l.undistorted, _options.slam);
        slam.frame[1].r.pyramid = utils::pyramid(slam.frame[1].r.undistorted, _options.slam);

        // track keypoints temporallyly
        utils::track(slam.frame[0].l, slam.frame[1].l, _options.slam);
        utils::track(slam.frame[0].r, slam.frame[1].r, _options.slam);

        SPDLOG_INFO("KLT tracked {} keypoints from previous frame in L", slam.frame[1].l.keypoints.size());
        SPDLOG_INFO("KLT tracked {} keypoints from previous frame in R", slam.frame[1].r.keypoints.size());

        // detect keypoints additional
        std::chrono::system_clock::duration detection_time { };
        {
            time_this time_this { detection_time };
            detector.detect(slam.frame[1].l.undistorted, slam.frame[1].l.keypoints);
            detector.detect(slam.frame[1].r.undistorted, slam.frame[1].r.keypoints);
        }

        SPDLOG_INFO("Detected points L: {}", slam.frame[1].l.keypoints.size());
        SPDLOG_INFO("Detected points R: {}", slam.frame[1].r.keypoints.size());
        SPDLOG_INFO("Detection time: {} s", std::chrono::duration<double>(detection_time).count());

        // match keypoints spatial
        utils::match(slam.frame[1].l.keypoints, slam.frame[1].r.keypoints, fundamental, _options.slam.threshold_epipolar);

        // Before we compute 3D-2D pose, we should compute the 3D-3D pose
        utils::triangulate(slam.frame[1], projection_L, projection_R, slam.frame[1].points);
        SPDLOG_INFO("Triangulated points count: {}", slam.frame[1].points.size());

        try
        {
            const auto &pose_data = utils::estimate_pose_3d3d
            (
                slam.frame[0].points,
                slam.frame[1].points,
                _options.slam.threshold_3d3d
            );
        }
        catch (const std::exception &error)
        {
            SPDLOG_WARN("{}", error.what());
        }

        try
        {
            const auto &pose_data = utils::estimate_pose_3d2d
            (
                slam.frame[0].points,
                slam.frame[1].l.keypoints,
                camera_matrix_L,
                _options.slam.threshold_3d2d
            );
        }
        catch (const std::runtime_error &error)
        {
            SPDLOG_WARN("{}", error.what());
        }

        // Gather points from slam.frame[0] and slam.frame[1] for 3D-3D pose computation
        std::vector<cv::Point3d> points3d_0 { };
        std::vector<cv::Point3d> points3d_1 { };
        std::vector<size_t>      indexes { };
        utils::correspondences_3d3d(slam.frame[0].points, slam.frame[1].points, points3d_0, points3d_1, indexes);

        // Compute relative pose between slam.frame[0] and slam.frame[1] using 3D-3D correspondences
        if (points3d_0.size() >= 3)
        {
            SPDLOG_INFO("Computing 3D-3D pose with {} correspondences", points3d_0.size());

            cv::Matx33d         R;
            cv::Point3d         t;
            std::vector<size_t> inliers { };
            std::vector<size_t> outliers { };
            std::vector<double> errors { };
            utils::estimate_rigid_ransac(points3d_0, points3d_1, R, t, inliers, outliers, errors, 0.01, 1000);

            SPDLOG_INFO("3D-3D pose inliers: {}", inliers.size());

            for (auto o: outliers)
            {
                slam.frame[1].points.erase(indexes[o]);
                slam.frame[0].points.erase(indexes[o]);
                slam.points.erase(indexes[o]);
            }

            cv::Affine3d pose { R, t };

            auto mean = 0.0;
            for (auto i: inliers)
            {
                mean += errors[i];
            }
            mean /= gsl::narrow<double>(inliers.size());

            SPDLOG_INFO("Reprojection error: {}", mean);

            if (mean < 0.01)
            {
                SPDLOG_INFO("Reprojection error is below threshold. Using pose from 3D-3D.");
                slam.frame[1].pose = slam.frame[0].pose * calibrations[0].pose_in_imu0 * pose.inv() * calibrations[0].
                                     pose_in_imu0.inv();
                SPDLOG_INFO("Pose: {}", slam.frame[1].pose);
            }
            else
            {
                SPDLOG_INFO("Reprojection error is above threshold. Using PnP.");
                std::vector<cv::Point3d> points3d;
                std::vector<cv::Point2d> points2d;
                std::vector<size_t>      indices = { };
                utils::correspondences_3d2d(slam.frame[0].points, slam.frame[1].l.keypoints, points3d, points2d, indices);

                if (points3d.size() >= 6)
                {
                    pose = slam.frame[1].pose.inv() * slam.frame[0].pose;

                    try
                    {
                        utils::solve_pnp(camera_matrix_L, points3d, points2d, pose);
                        slam.frame[1].pose = slam.frame[0].pose * calibrations[0].pose_in_imu0 * pose.inv() * calibrations[0].
                                             pose_in_imu0.inv();
                        SPDLOG_INFO("Pose: {}", slam.frame[1].pose);
                    }
                    catch (std::exception &e)
                    {
                        SPDLOG_WARN("SolvePnP failed: {}", e.what());
                    }
                }
                else
                {
                    SPDLOG_WARN("Not enough points for PnP: {}", points3d.size());
                }
            }
        }

        for (const auto &[index, point]: slam.frame[1].points)
        {
            const auto &image_point = slam.frame[1].l.keypoints.at(point.index).pt;
            const auto &pixel       = slam.frame[1].l.undistorted.at<cv::Vec3b>(image_point);

            auto point3d  = slam.frame[1].pose * calibrations[0].pose_in_imu0 * point;
            point3d.index = index;
            point3d.color = pixel;
            slam.points.emplace(index, point3d);
        }

        SPDLOG_INFO("Groundtruth pose: {}", slam.frame[1].pose_gt);

        SPDLOG_INFO("Map points count: {}", slam.points.size());

        slam.colors = slam.points | std::views::values | std::views::transform
                      (
                          [](const auto &p)
                          {
                              return p.color;
                          }
                      ) | std::ranges::to<std::vector>();

        motion.update(slam.frame[0].pose, slam.frame[1].pose, dt);

        on_frame(slam);

        if (_stop_token.stop_requested())
        {
            break;
        }
    }
}
