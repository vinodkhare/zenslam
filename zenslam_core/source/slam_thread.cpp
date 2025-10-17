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
#include "time_this.h"
#include "utils.h"
#include "utils_opencv.h"
#include "utils_slam.h"
#include "frame/durations.h"
#include "frame/slam.h"
#include "frame/writer.h"

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
    auto writer      = frame::writer(_options.folder.output / "frame_data.csv");

    calibration.print();

    zenslam::frame::slam slam { };

    for (auto f: stereo_reader)
    {
        {
            slam.frames[0] = std::move(slam.frames[1]);
            slam.frames[1] = std::move(f);

            time_this t { slam.durations.total };

            const auto &dt = isnan(slam.frames[0].cameras[0].timestamp)
                                 ? 0.0
                                 : slam.frames[1].cameras[0].timestamp - slam.frames[0].cameras[0].timestamp;
            const auto &slerp                    = groundtruth.slerp(slam.frames[1].cameras[0].timestamp);
            const auto &pose_gt_of_imu0_in_world = cv::Affine3d { slerp.quaternion.toRotMat3x3(), slerp.translation };

            slam.frames[1].pose_gt = pose_gt_of_imu0_in_world * calibration.cameras[0].pose_in_imu0;
            slam.frames[0].pose    = isnan(slam.frames[0].cameras[0].timestamp) ? slam.frames[1].pose_gt : slam.frames[0].pose;
            slam.frames[1].pose    = motion.predict(slam.frames[0].pose, dt);

            // PREPROCESS
            {
                time_this time_this { slam.durations.preprocessing };

                slam.frames[1] = utils::pre_process(slam.frames[1], calibration.cameras, _options.slam, clahe);
            }

            // TRACK
            {
                time_this time_this { slam.durations.tracking };

                const auto &tracked_keypoints = utils::track(slam.frames, _options.slam);
                slam.frames[1].cameras[0].keypoints += tracked_keypoints[0];
                slam.frames[1].cameras[1].keypoints += tracked_keypoints[1];

                slam.frames[1].cameras[0].keylines += utils::track_keylines
                (
                    slam.frames[0].cameras[0].pyramid,
                    slam.frames[1].cameras[0].pyramid,
                    slam.frames[0].cameras[0].keylines,
                    _options.slam
                );

                slam.frames[1].cameras[1].keylines += utils::track_keylines
                (
                    slam.frames[0].cameras[1].pyramid,
                    slam.frames[1].cameras[1].pyramid,
                    slam.frames[0].cameras[1].keylines,
                    _options.slam
                );
            }

            slam.counts.keypoints_l_tracked = slam.frames[1].cameras[0].keypoints.size();
            slam.counts.keypoints_r_tracked = slam.frames[1].cameras[1].keypoints.size();

            // DETECT
            {
                time_this time_this { slam.durations.detection };

                slam.frames[1].cameras[0].keypoints += detector.detect
                        (slam.frames[1].cameras[0].undistorted, slam.frames[1].cameras[0].keypoints);

                slam.frames[1].cameras[1].keypoints += detector.detect
                        (slam.frames[1].cameras[1].undistorted, slam.frames[1].cameras[1].keypoints);

                slam.frames[1].cameras[0].keylines += detector.detect
                        (slam.frames[1].cameras[0].undistorted, slam.frames[1].cameras[0].keylines);

                slam.frames[1].cameras[1].keylines += detector.detect
                        (slam.frames[1].cameras[1].undistorted, slam.frames[1].cameras[1].keylines);
            }

            // MATCH & TRIANGULATE
            {
                time_this time_this { slam.durations.matching };

                slam.frames[1].cameras[1].keypoints *= utils::match
                (
                    slam.frames[1].cameras[0].keypoints,
                    slam.frames[1].cameras[1].keypoints,
                    calibration.fundamental_matrix[0],
                    _options.slam.threshold_epipolar
                );

                std::vector<double> errors { };
                std::tie(slam.frames[1].points, errors) = utils::triangulate
                (
                    slam.frames[1],
                    calibration.projection_matrix[0],
                    calibration.projection_matrix[1],
                    _options.slam.threshold_triangulate
                );

                slam.counts.maches_triangulated = slam.frames[1].points.size();
                slam.counts.matches             = std::ranges::count_if
                (
                    slam.frames[1].cameras[0].keypoints | std::views::keys,
                    [&slam](const auto &index)
                    {
                        return slam.frames[1].cameras[1].keypoints.contains(index);
                    }
                );

                SPDLOG_INFO("");
                SPDLOG_INFO
                (
                    "Tri mean error:   {:.4f} px",
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
                            utils::estimate_pose_3d3d
                            (slam.frames[0].points, slam.frames[1].points, _options.slam.threshold_3d3d);
                }
                catch (const std::runtime_error &error)
                {
                    SPDLOG_WARN("Unable to estimate pose because: {}", error.what());
                }

                try
                {
                    pose_data_3d2d = utils::estimate_pose_3d2d
                    (
                        slam.frames[0].points,
                        slam.frames[1].cameras[0].keypoints,
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

            SPDLOG_INFO("3D-3D mean error: {:.4f} m", utils::mean(pose_data_3d3d.errors));
            SPDLOG_INFO("3D-2D mean error: {:.4f} px", utils::mean(pose_data_3d2d.errors));

            const auto &pose =
                    pose_data_3d3d.inliers.size() > pose_data_3d2d.inliers.size() ? pose_data_3d3d.pose : pose_data_3d2d.pose;

            SPDLOG_INFO("");
            SPDLOG_INFO("Predicted pose:   {}", slam.frames[1].pose);

            slam.frames[1].pose = slam.frames[0].pose * pose.inv();

            SPDLOG_INFO("Estimated pose:   {}", slam.frames[1].pose);
            SPDLOG_INFO("Groundtruth pose: {}", slam.frames[1].pose_gt);

            for (const auto &[index, point]: slam.frames[1].points)
            {
                auto point3d = slam.frames[1].pose * point;

                point3d.index = index;
                point3d.color = slam.frames[1].cameras[0].undistorted.at<cv::Vec3b>
                        (slam.frames[1].cameras[0].keypoints.at(point.index).pt);

                slam.points[index] = point3d;
            }

            slam.counts.points = slam.points.size();

            slam.colors = slam.points | std::views::values | std::views::transform
                          (
                              [](const auto &p)
                              {
                                  return p.color;
                              }
                          ) |
                          std::ranges::to<std::vector>();

            motion.update(slam.frames[0].pose, slam.frames[1].pose, dt);

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
