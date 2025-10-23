#include "slam_thread.h"


#include <utility>

#include <gsl/narrow>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <spdlog/spdlog.h>

#include <vtk-9.3/vtkLogger.h>

#include "calibration.h"
#include "grid_detector.h"
#include "groundtruth.h"
#include "motion.h"
#include "stereo_folder_reader.h"
#include "time_this.h"
#include "utils.h"
#include "utils_slam.h"
#include "utils_std.h"
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
    const auto& calibration   = calibration::parse(_options.folder.calibration_file, _options.folder.imu_calibration_file, _options.slam.stereo_rectify);
    const auto& stereo_reader = stereo_folder_reader(_options.folder);
    const auto& clahe         = cv::createCLAHE(4.0); // TODO: make configurable
    const auto& detector      = grid_detector::create(_options.slam);

    auto groundtruth = groundtruth::read(_options.folder.groundtruth_file);
    auto motion      = zenslam::motion();
    auto writer      = frame::writer(_options.folder.output / "frame_data.csv");

    calibration.print();

    frame::slam slam { };

    for (auto f: stereo_reader)
    {
        {
            slam.frames[0] = std::move(slam.frames[1]);
            slam.frames[1] = std::move(f);

            time_this t { slam.durations.total };

            const auto& dt = isnan(slam.frames[0].cameras[0].timestamp)
                                 ? 0.0
                                 : slam.frames[1].cameras[0].timestamp - slam.frames[0].cameras[0].timestamp;
            const auto& slerp                    = groundtruth.slerp(slam.frames[1].cameras[0].timestamp);
            const auto& pose_gt_of_imu0_in_world = cv::Affine3d { slerp.quaternion.toRotMat3x3(), slerp.translation };

            slam.frames[1].pose_gt = pose_gt_of_imu0_in_world * calibration.cameras[0].pose_in_imu0;
            slam.frames[0].pose    = isnan(slam.frames[0].cameras[0].timestamp) ? slam.frames[1].pose_gt : slam.frames[0].pose;
            slam.frames[1].pose    = motion.predict(slam.frames[0].pose, dt);

            // PREPROCESS
            {
                time_this time_this { slam.durations.preprocessing };

                slam.frames[1] = utils::pre_process(slam.frames[1], calibration, _options.slam, clahe);
            }

            // TODO: separate keyline and keypoints pipelines
            // TRACK
            {
                time_this time_this { slam.durations.tracking };

                slam.frames[1].cameras[0].keypoints += utils::track_keypoints
                (
                    slam.frames[0].cameras[0].pyramid,
                    slam.frames[1].cameras[0].pyramid,
                    slam.frames[0].cameras[0].keypoints,
                    _options.slam,
                    calibration.camera_matrix[0]
                );

                slam.frames[1].cameras[1].keypoints += utils::track_keypoints
                (
                    slam.frames[0].cameras[1].pyramid,
                    slam.frames[1].cameras[1].pyramid,
                    slam.frames[0].cameras[1].keypoints,
                    _options.slam,
                    calibration.camera_matrix[1]
                );

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

                slam.frames[1].cameras[0].keypoints += detector.detect_keypoints
                        (slam.frames[1].cameras[0].undistorted, slam.frames[1].cameras[0].keypoints);

                slam.frames[1].cameras[1].keypoints += detector.detect_keypoints
                        (slam.frames[1].cameras[1].undistorted, slam.frames[1].cameras[1].keypoints);

                slam.frames[1].cameras[0].keylines += detector.detect_keylines
                        (slam.frames[1].cameras[0].undistorted, slam.frames[1].cameras[0].keylines);

                slam.frames[1].cameras[1].keylines += detector.detect_keylines
                        (slam.frames[1].cameras[1].undistorted, slam.frames[1].cameras[1].keylines);
            }

            slam.counts.keypoints_l = slam.frames[1].cameras[0].keypoints.size();
            slam.counts.keypoints_r = slam.frames[1].cameras[1].keypoints.size();

            // MATCH & TRIANGULATE
            {
                time_this time_this { slam.durations.matching };

                // find keypoints that are not triangulated
                // {
                //     const auto& matches3d = utils::match_keypoints3d
                //     (
                //         slam.points3d,
                //         slam.frames[1].cameras[0].keypoints,
                //         slam.frames[1].pose,
                //         calibration.cameras[0].projection(slam.frames[1].pose),
                //         _options.slam.triangulation_max_depth,
                //         _options.slam.triangulation_reprojection_threshold
                //     );
                //
                //     slam.frames[1].cameras[0].keypoints *= matches3d;
                // }
                //
                // {
                //     const auto& matches3d = utils::match_keypoints3d
                //     (
                //         slam.points3d,
                //         slam.frames[1].cameras[1].keypoints,
                //         slam.frames[1].pose,
                //         calibration.cameras[0].projection(slam.frames[1].pose),
                //         _options.slam.triangulation_max_depth,
                //         _options.slam.triangulation_reprojection_threshold
                //     );
                //
                //     slam.frames[1].cameras[1].keypoints *= matches3d;
                // }


                auto matches = utils::match_keypoints
                (
                    slam.frames[1].cameras[0].keypoints,
                    slam.frames[1].cameras[1].keypoints,
                    calibration.fundamental_matrix[0],
                    _options.slam.epipolar_threshold
                );

                slam.frames[1].cameras[1].keypoints *= matches;

                slam.counts.matches = matches.size();

                slam.frames[1].points3d += utils::triangulate_keypoints
                (
                    slam.frames[1].cameras[0].keypoints,
                    slam.frames[1].cameras[1].keypoints,
                    calibration.projection_matrix[0],
                    calibration.projection_matrix[1],
                    _options.slam.triangulation_reprojection_threshold,
                    calibration.cameras[1].pose_in_cam0.translation()
                );

                slam.counts.maches_triangulated = slam.frames[1].points3d.size();

                slam.frames[1].cameras[1].keylines *= utils::match_keylines
                (
                    slam.frames[1].cameras[0].keylines,
                    slam.frames[1].cameras[1].keylines,
                    calibration.fundamental_matrix[0],
                    _options.slam.epipolar_threshold
                );

                slam.frames[1].lines3d += utils::triangulate_keylines
                (
                    slam.frames[1].cameras[0].keylines,
                    slam.frames[1].cameras[1].keylines,
                    calibration.projection_matrix[0],
                    calibration.projection_matrix[1],
                    _options.slam,
                    calibration.cameras[1].pose_in_cam0.translation()
                );
            }

            auto pose_data_3d3d = pose_data { };
            auto pose_data_3d2d = pose_data { };

            // ESTIMATE
            {
                time_this time_this { slam.durations.estimation };

                try
                {
                    pose_data_3d3d = utils::estimate_pose_3d3d
                    (
                        slam.frames[0].points3d,
                        slam.frames[1].points3d,
                        _options.slam.threshold_3d3d
                    );
                }
                catch (const std::runtime_error& error)
                {
                    SPDLOG_WARN("Unable to estimate pose because: {}", error.what());
                }

                try
                {
                    pose_data_3d2d = utils::estimate_pose_3d2d
                    (
                        slam.frames[0].points3d,
                        slam.frames[1].cameras[0].keypoints,
                        calibration.camera_matrix[0],
                        _options.slam.threshold_3d2d
                    );
                }
                catch (const std::runtime_error& error)
                {
                    SPDLOG_WARN("Unable to estimate pose because: {}", error.what());
                }
            }

            slam.counts.correspondences_3d2d         = pose_data_3d2d.indices.size();
            slam.counts.correspondences_3d3d         = pose_data_3d3d.indices.size();
            slam.counts.correspondences_3d2d_inliers = pose_data_3d2d.inliers.size();
            slam.counts.correspondences_3d3d_inliers = pose_data_3d3d.inliers.size();

            const auto err3d3d_mean = utils::mean(pose_data_3d3d.errors);
            const auto err3d2d_mean = utils::mean(pose_data_3d2d.errors);
            SPDLOG_INFO("3D-3D mean error: {:.4f} m", err3d3d_mean);
            SPDLOG_INFO("3D-2D mean error: {:.4f} px", err3d2d_mean);

            const auto& pose =
                    pose_data_3d3d.inliers.size() > pose_data_3d2d.inliers.size() ? pose_data_3d3d.pose : pose_data_3d2d.pose;

            SPDLOG_INFO("");
            SPDLOG_INFO("Predicted pose:   {}", slam.frames[1].pose);

            slam.frames[1].pose = slam.frames[0].pose * pose.inv();

            SPDLOG_INFO("Estimated pose:   {}", slam.frames[1].pose);
            SPDLOG_INFO("Groundtruth pose: {}", slam.frames[1].pose_gt);

            slam.points3d += slam.frames[1].pose * slam.frames[1].points3d;
            slam.lines3d += slam.frames[1].pose * slam.frames[1].lines3d;

            slam.points3d.buildIndex();

            slam.counts.points = slam.points3d.size();

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
