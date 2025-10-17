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
#include "frame_durations.h"
#include "frame_writer.h"
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
    const auto &calibration   = calibration::parse(_options.folder.calibration_file, _options.folder.imu_calibration_file);
    const auto &stereo_reader = stereo_folder_reader(_options.folder);
    const auto &clahe         = cv::createCLAHE();
    const auto &detector      = grid_detector::create(_options.slam);

    auto groundtruth = groundtruth::read(_options.folder.groundtruth_file);
    auto motion      = zenslam::motion();
    auto writer      = frame_writer(_options.folder.output / "frame_data.csv");

    calibration.print();

    slam_frame slam { };

    for (auto f: stereo_reader)
    {
        {
            slam.frame[0] = std::move(slam.frame[1]);
            slam.frame[1] = std::move(f);

            time_this t { slam.durations.total };

            const auto &dt = isnan(slam.frame[0].l.timestamp) ? 0.0 : slam.frame[1].l.timestamp - slam.frame[0].l.timestamp;
            const auto &slerp = groundtruth.slerp(slam.frame[1].l.timestamp);
            const auto &pose_gt_of_imu0_in_world = cv::Affine3d { slerp.quaternion.toRotMat3x3(), slerp.translation };

            slam.frame[1].pose_gt = pose_gt_of_imu0_in_world * calibration.camera[0].pose_in_imu0;
            slam.frame[0].pose    = isnan(slam.frame[0].l.timestamp) ? slam.frame[1].pose_gt : slam.frame[0].pose;
            slam.frame[1].pose    = motion.predict(slam.frame[0].pose, dt);

            SPDLOG_INFO("");
            SPDLOG_INFO("Predicted pose: {}", slam.frame[1].pose);

            // PREPROCESS
            {
                time_this time_this { slam.durations.preprocessing };

                cv::cvtColor(slam.frame[1].l.image, slam.frame[1].l.image, cv::COLOR_BGR2GRAY);
                cv::cvtColor(slam.frame[1].r.image, slam.frame[1].r.image, cv::COLOR_BGR2GRAY);

                if (_options.slam.clahe_enabled)
                {
                    clahe->apply(slam.frame[1].l.image, slam.frame[1].l.image);
                    clahe->apply(slam.frame[1].r.image, slam.frame[1].r.image);
                }

                slam.frame[1].l.undistorted = utils::undistort(slam.frame[1].l.image, calibration.camera[0]);
                slam.frame[1].r.undistorted = utils::undistort(slam.frame[1].r.image, calibration.camera[1]);

                slam.frame[1].l.pyramid = utils::pyramid(slam.frame[1].l.undistorted, _options.slam);
                slam.frame[1].r.pyramid = utils::pyramid(slam.frame[1].r.undistorted, _options.slam);
            }

            // TRACK
            {
                time_this time_this { slam.durations.tracking };

                const auto &projection           = calibration.camera[0].projection(slam.frame[1].pose);
                auto        points_1_l_predicted = std::vector<cv::Point2f> { };
                auto        points_1_r_predicted = std::vector<cv::Point2f> { };

                for (auto &[index, keypoint]: slam.frame[0].l.keypoints)
                {
                    points_1_l_predicted.emplace_back(keypoint.pt);

                    if (slam.points.contains(index))
                    {
                        auto points2d = utils::project
                        (
                            { static_cast<cv::Point3d>(slam.points[index]) },
                            projection
                        );

                        points_1_l_predicted.back() = points2d[0];
                    }
                }

                for (auto &[index, keypoint]: slam.frame[0].r.keypoints)
                {
                    points_1_r_predicted.emplace_back(keypoint.pt);

                    if (slam.points.contains(index))
                    {
                        auto point2d = utils::project
                        (
                            { static_cast<cv::Point3d>(slam.points[index]) },
                            calibration.camera[1].projection(slam.frame[1].pose)
                        );
                    }
                }

                utils::track(slam.frame[0].l, slam.frame[1].l, _options.slam, points_1_l_predicted);
                utils::track(slam.frame[0].r, slam.frame[1].r, _options.slam, points_1_r_predicted);
            }

            slam.counts.keypoints_l_tracked = slam.frame[1].l.keypoints.size();
            slam.counts.keypoints_r_tracked = slam.frame[1].r.keypoints.size();

            // DETECT
            {
                time_this time_this { slam.durations.detection };

                detector.detect(slam.frame[1].l.undistorted, slam.frame[1].l.keypoints);
                detector.detect(slam.frame[1].r.undistorted, slam.frame[1].r.keypoints);
            }

            slam.counts.keypoints_l = slam.frame[1].l.keypoints.size();
            slam.counts.keypoints_r = slam.frame[1].r.keypoints.size();

            // MATCH & TRIANGULATE
            {
                time_this time_this { slam.durations.matching };

                auto matches_temporal = utils::match_temporal
                (
                    slam.frame[0].l.keypoints,
                    slam.frame[1].l.keypoints,
                    calibration.camera_matrix[0],
                    0.1
                );

                for (const auto &match: matches_temporal)
                {
                    const auto keypoint_0 = slam.frame[0].l.keypoints.at(match.queryIdx);
                    const auto keypoint_1 = slam.frame[1].l.keypoints.at(match.trainIdx);

                    slam.frame[1].l.keypoints.erase(keypoint_1.index);

                    slam.frame[1].l.keypoints[keypoint_0.index]       = keypoint_1;
                    slam.frame[1].l.keypoints[keypoint_0.index].index = keypoint_0.index;
                }

                matches_temporal = utils::match_temporal
                (
                    slam.frame[0].r.keypoints,
                    slam.frame[1].r.keypoints,
                    calibration.camera_matrix[1],
                    0.1
                );

                for (const auto &match: matches_temporal)
                {
                    const auto keypoint_0 = slam.frame[0].r.keypoints.at(match.queryIdx);
                    const auto keypoint_1 = slam.frame[1].r.keypoints.at(match.trainIdx);

                    slam.frame[1].r.keypoints.erase(keypoint_1.index);

                    slam.frame[1].r.keypoints[keypoint_0.index]       = keypoint_1;
                    slam.frame[1].r.keypoints[keypoint_0.index].index = keypoint_0.index;
                }

                const auto &matches_spatial = utils::match
                (
                    slam.frame[1].l.keypoints,
                    slam.frame[1].r.keypoints,
                    calibration.fundamental_matrix[0],
                    _options.slam.threshold_epipolar
                );

                for (const auto &match: matches_spatial)
                {
                    const auto keypoint_l = slam.frame[1].l.keypoints.at(match.queryIdx);
                    const auto keypoint_r = slam.frame[1].r.keypoints.at(match.trainIdx);

                    slam.frame[1].r.keypoints.erase(keypoint_r.index);

                    slam.frame[1].r.keypoints[keypoint_l.index]       = keypoint_r;
                    slam.frame[1].r.keypoints[keypoint_l.index].index = keypoint_l.index;
                }

                std::vector<double> errors { };
                std::tie(slam.frame[1].points, errors) = utils::triangulate
                (
                    slam.frame[1],
                    calibration.projection_matrix[0],
                    calibration.projection_matrix[1]
                );

                slam.counts.maches_triangulated = slam.frame[1].points.size();
                slam.counts.matches             = std::ranges::count_if
                (
                    slam.frame[1].l.keypoints | std::views::keys,
                    [&slam](const auto &index)
                    {
                        return slam.frame[1].r.keypoints.contains(index);
                    }
                );

                SPDLOG_INFO
                (
                    "Triangulation mean error: {} pixels",
                    utils::mean
                    (
                        errors | std::views::filter
                        (
                            [this](const auto &e) { return e <= _options.slam.threshold_triangulate; }
                        ) | std::ranges::to<std::vector>()
                    )
                );
            }

            auto pose_data_3d3d = pose_data { };
            auto pose_data_3d2d = pose_data { };

            // ESTIMATE
            {
                time_this time_this { slam.durations.estimation };

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
                        calibration.camera_matrix[0],
                        _options.slam.threshold_3d2d
                    );
                }
                catch (const std::runtime_error &error)
                {
                    SPDLOG_WARN("Unable to estimate pose because: {}", error.what());
                }
            }

            slam.counts.correspondences_3d2d         = pose_data_3d2d.indices.size();
            slam.counts.correspondences_3d3d         = pose_data_3d3d.indices.size();
            slam.counts.correspondences_3d2d_inliers = pose_data_3d2d.inliers.size();
            slam.counts.correspondences_3d3d_inliers = pose_data_3d3d.inliers.size();

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

            slam.counts.print();
            slam.durations.print();

            writer.write(slam);

            on_frame(slam);

            if (_stop_token.stop_requested())
            {
                break;
            }
        }
    }
}
