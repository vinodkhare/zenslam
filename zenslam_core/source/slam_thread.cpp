#include "slam_thread.h"


#include <utility>

#include <gsl/narrow>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

#include <spdlog/spdlog.h>

#include <vtk-9.3/vtkLogger.h>

#include "calibration.h"
#include "grid_detector.h"
#include "motion.h"
#include "utils.h"
#include "utils_slam.h"
#include "time_this.h"

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
    auto        motion            = zenslam::motion();

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

    stereo_frame            frame_0 { };
    std::map<size_t, point> points { };

    for (auto frame_1: stereo_reader)
    {
        auto dt = isnan(frame_0.l.timestamp) ? 0.0 : frame_1.l.timestamp - frame_0.l.timestamp;

        frame_1.pose = motion.predict(frame_0.pose, dt);

        frame_1.l.undistorted = utils::undistort(frame_1.l.image, calibrations[0]);
        frame_1.r.undistorted = utils::undistort(frame_1.r.image, calibrations[1]);

        frame_1.l.pyramid = utils::pyramid(frame_1.l.undistorted, _options.slam);
        frame_1.r.pyramid = utils::pyramid(frame_1.r.undistorted, _options.slam);

        // track keypoints temporallyly
        utils::track(frame_0.l, frame_1.l, _options.slam);
        utils::track(frame_0.r, frame_1.r, _options.slam);

        SPDLOG_INFO("KLT tracked {} keypoints from previous frame in L", frame_1.l.keypoints.size());
        SPDLOG_INFO("KLT tracked {} keypoints from previous frame in R", frame_1.r.keypoints.size());

        // detect keypoints additional
        std::chrono::system_clock::duration detection_time {};
        {
            time_this time_this { detection_time };
            detector.detect_par(frame_1.l.undistorted, frame_1.l.keypoints);
            detector.detect_par(frame_1.r.undistorted, frame_1.r.keypoints);
        }

        SPDLOG_INFO("Detected points L: {}", frame_1.l.keypoints.size());
        SPDLOG_INFO("Detected points R: {}", frame_1.r.keypoints.size());
        SPDLOG_INFO("Detection time: {} s", std::chrono::duration<double>(detection_time).count());

        // match keypoints spatial
        utils::match(frame_1.l.keypoints, frame_1.r.keypoints, fundamental, _options.slam.epipolar_threshold);

        // Before we compute 3D-2D pose, we should compute the 3D-3D pose
        utils::triangulate(frame_1, projection_L, projection_R, frame_1.points);
        SPDLOG_INFO("Triangulated points count: {}", frame_1.points.size());

        // Gather points from frame_0 and frame_1 for 3D-3D pose computation
        std::vector<cv::Point3d> points3d_0 { };
        std::vector<cv::Point3d> points3d_1 { };
        std::vector<size_t>      indexes { };
        utils::correspondences_3d3d(frame_0.points, frame_1.points, points3d_0, points3d_1, indexes);

        SPDLOG_DEBUG("Predicted pose: {}", frame_1.pose);

        // Compute relative pose between frame_0 and frame_1 using 3D-3D correspondences
        if (points3d_0.size() >= 3)
        {
            SPDLOG_INFO("Computing 3D-3D pose with {} correspondences", points3d_0.size());

            cv::Matx33d         R;
            cv::Point3d         t;
            std::vector<size_t> inliers { };
            std::vector<size_t> outliers { };
            std::vector<double> errors { };
            utils::estimate_rigid_ransac(points3d_0, points3d_1, R, t, inliers, outliers, errors, 0.01, 1000, 3);

            SPDLOG_INFO("3D-3D pose inliers: {}", inliers.size());

            for (auto o: outliers)
            {
                frame_1.points.erase(indexes[o]);
                frame_0.points.erase(indexes[o]);
                points.erase(indexes[o]);
            }

            cv::Affine3d pose { R, t };

            auto mean = 0.0;
            for (auto i: inliers)
            {
                mean += errors[i];
            }
            mean /= inliers.size();

            SPDLOG_INFO("Reprojection error: {}", mean);

            if (mean < 0.01)
            {
                SPDLOG_INFO("Reprojection error is below threshold. Using pose from 3D-3D.");
                frame_1.pose = frame_0.pose * pose.inv();
                SPDLOG_INFO("Pose: {}", frame_1.pose);
            }
            else
            {
                SPDLOG_INFO("Reprojection error is above threshold. Using PnP.");
                std::vector<cv::Point3d> points3d;
                std::vector<cv::Point2d> points2d;
                utils::correspondences(frame_0.points, frame_1.l.keypoints, points3d, points2d);

                if (points3d.size() >= 6)
                {
                    pose = frame_1.pose.inv() * frame_0.pose;

                    try
                    {
                        utils::solve_pnp(camera_matrix_L, points3d, points2d, pose);
                        frame_1.pose = frame_0.pose * pose.inv();
                        SPDLOG_INFO("Pose: {}", frame_1.pose);
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

        for (auto [index, point]: frame_1.points)
        {
            auto point3d  = frame_1.pose * point;
            point3d.index = index;
            points.emplace(index, point3d);
        }

        SPDLOG_INFO("Map points count: {}", points.size());

        frame_1.points3d = points | std::views::values | std::views::transform
                           (
                               [](const auto &p)-> cv::Point3d
                               {
                                   return p;
                               }
                           ) |
                           std::ranges::to<std::vector>();

        motion.update(frame_0.pose, frame_1.pose, dt);

        on_frame(frame_0 = std::move(frame_1));

        if (_stop_token.stop_requested())
        {
            break;
        }
    }
}
