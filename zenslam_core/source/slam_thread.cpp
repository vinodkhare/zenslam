#include "slam_thread.h"


#include <utility>

#include <gsl/narrow>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>
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
    const auto &stereo_reader = stereo_folder_reader(_options.folder);
    const auto &clahe         = cv::createCLAHE();
    const auto &detector      = grid_detector::create(_options.slam);

    auto groundtruth = groundtruth::read(_options.folder.groundtruth_file);
    auto motion      = zenslam::motion();

    const auto &calibrations = std::vector {
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
        std::chrono::system_clock::duration frame_duration { };
        {
            time_this t { frame_duration };

            slam.frame[0] = std::move(slam.frame[1]);
            slam.frame[1] = std::move(f);

            const auto &dt = isnan(slam.frame[0].l.timestamp) ? 0.0 : slam.frame[1].l.timestamp - slam.frame[0].l.timestamp;
            const auto &slerp = groundtruth.slerp(slam.frame[1].l.timestamp);
            const auto &pose_gt_of_imu0_in_world = cv::Affine3d { slerp.quaternion.toRotMat3x3(), slerp.translation };

            slam.frame[1].pose_gt = pose_gt_of_imu0_in_world * calibrations[0].pose_in_imu0;
            slam.frame[0].pose    = isnan(slam.frame[0].l.timestamp) ? slam.frame[1].pose_gt : slam.frame[0].pose;
            slam.frame[1].pose    = motion.predict(slam.frame[0].pose, dt);

            SPDLOG_INFO("");
            SPDLOG_INFO("Predicted pose: {}", slam.frame[1].pose);

            std::chrono::system_clock::duration preprocessing_duration { };
            {
                time_this time_this { preprocessing_duration };

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
            }
            SPDLOG_INFO("Preprocessing duration: {} s", std::chrono::duration<double>(preprocessing_duration).count());

            // track keypoints temporallyly
            std::chrono::system_clock::duration tracking_duration { };
            {
                time_this time_this { tracking_duration };

                utils::track(slam.frame[0].l, slam.frame[1].l, _options.slam);
                utils::track(slam.frame[0].r, slam.frame[1].r, _options.slam);
            }
            SPDLOG_INFO("Tracking duration: {} s", std::chrono::duration<double>(tracking_duration).count());

            SPDLOG_INFO("KLT tracked {} keypoints from previous frame in L", slam.frame[1].l.keypoints.size());
            SPDLOG_INFO("KLT tracked {} keypoints from previous frame in R", slam.frame[1].r.keypoints.size());

            // detect keypoints additional
            std::chrono::system_clock::duration detection_duration { };
            {
                time_this time_this { detection_duration };

                detector.detect(slam.frame[1].l.undistorted, slam.frame[1].l.keypoints);
                detector.detect(slam.frame[1].r.undistorted, slam.frame[1].r.keypoints);
            }
            SPDLOG_INFO("Detection duration: {} s", std::chrono::duration<double>(detection_duration).count());

            SPDLOG_INFO("Detected points L: {}", slam.frame[1].l.keypoints.size());
            SPDLOG_INFO("Detected points R: {}", slam.frame[1].r.keypoints.size());

            // match keypoints spatial
            std::chrono::system_clock::duration matching_duration { };
            {
                time_this time_this { matching_duration };

                const auto &matches = utils::match
                (
                    slam.frame[1].l.keypoints,
                    slam.frame[1].r.keypoints,
                    fundamental,
                    _options.slam.threshold_epipolar
                );

                SPDLOG_INFO("Matched points: {}", matches.size());

                for (const auto &match: matches)
                {
                    const auto keypoint_l = slam.frame[1].l.keypoints.at(match.queryIdx);
                    const auto keypoint_r = slam.frame[1].r.keypoints.at(match.trainIdx);

                    slam.frame[1].r.keypoints.erase(keypoint_r.index);

                    slam.frame[1].r.keypoints[keypoint_l.index]       = keypoint_r;
                    slam.frame[1].r.keypoints[keypoint_l.index].index = keypoint_l.index;
                }

                std::vector<double> errors { };
                std::tie(slam.frame[1].points, errors) = utils::triangulate(slam.frame[1], projection_L, projection_R);

                SPDLOG_INFO("Triangulated point count: {}", slam.frame[1].points.size());
                SPDLOG_INFO
                (
                    "Triangulation mean error: {} pixels",
                    utils::mean(errors | std::views::filter([this](const auto &e) { return e <= _options.slam.
                        threshold_triangulate; }) | std::ranges::to<std::vector>())
                );
            }
            SPDLOG_INFO("Matching & triangulation duration: {} s", std::chrono::duration<double>(matching_duration).count());

            auto pose_data_3d3d = pose_data { };
            auto pose_data_3d2d = pose_data { };

            std::chrono::system_clock::duration pose_estimation_duration { };
            {
                time_this time_this { pose_estimation_duration };

                try
                {
                    pose_data_3d3d =
                            utils::estimate_pose_3d3d(slam.frame[0].points, slam.frame[1].points, _options.slam.threshold_3d3d);
                }
                catch (const std::runtime_error &error)
                {
                    SPDLOG_WARN("Unable to estimate pose because: {}", error.what());
                }

                try
                {
                    pose_data_3d2d = utils::estimate_pose_3d2d
                    (
                        slam.frame[0].points,
                        slam.frame[1].l.keypoints,
                        camera_matrix_L,
                        _options.slam.threshold_3d2d
                    );
                }
                catch (const std::runtime_error &error)
                {
                    SPDLOG_WARN("Unable to estimate pose because: {}", error.what());
                }
            }
            SPDLOG_INFO("Pose estimation duration: {} s", std::chrono::duration<double>(pose_estimation_duration).count());

            SPDLOG_INFO("");
            SPDLOG_INFO("3D-3D correspondences: {}", pose_data_3d3d.indices.size());
            SPDLOG_INFO("3D-3D inliers: {}", pose_data_3d3d.inliers.size());
            SPDLOG_INFO("3D-3D mean error: {} m", utils::mean(pose_data_3d3d.errors));

            SPDLOG_INFO("");
            SPDLOG_INFO("3D-2D correspondeces: {}", pose_data_3d2d.indices.size());
            SPDLOG_INFO("3D-2D inliers: {}", pose_data_3d2d.inliers.size());
            SPDLOG_INFO("3D-2D mean error: {} pixels", utils::mean(pose_data_3d2d.errors));

            const auto &pose =
                    pose_data_3d3d.inliers.size() > pose_data_3d2d.inliers.size() ? pose_data_3d3d.pose : pose_data_3d2d.pose;

            slam.frame[1].pose = slam.frame[0].pose * pose.inv();

            for (const auto &[index, point]: slam.frame[1].points)
            {
                auto point3d = slam.frame[1].pose * point;

                point3d.index = index;
                point3d.color = slam.frame[1].l.undistorted.at<cv::Vec3b>(slam.frame[1].l.keypoints.at(point.index).pt);

                slam.points[index] = point3d;
            }

            SPDLOG_INFO("");
            SPDLOG_INFO("Groundtruth pose: {}", slam.frame[1].pose_gt);
            SPDLOG_INFO("Estimated pose: {}", slam.frame[1].pose);
            SPDLOG_INFO("Map points count: {}", slam.points.size());

            slam.colors = slam.points | std::views::values | std::views::transform
                          (
                              [](const auto &p)
                              {
                                  return p.color;
                              }
                          ) |
                          std::ranges::to<std::vector>();

            motion.update(slam.frame[0].pose, slam.frame[1].pose, dt);

            on_frame(slam);

            if (_stop_token.stop_requested())
            {
                break;
            }
        }
        SPDLOG_INFO("Frame duration: {} s", std::chrono::duration<double>(frame_duration).count());
    }
}
