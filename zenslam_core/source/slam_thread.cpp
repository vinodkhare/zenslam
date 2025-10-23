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
#include "frame/system.h"
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

    frame::system system { };

    for (auto stereo: stereo_reader)
    {
        {
            system[0] = std::move(system[1]);

            auto sensor = frame::sensor { 0, stereo.cameras[0].timestamp, { stereo.cameras[0].image, stereo.cameras[1].image } };

            time_this t { system.durations.total };

            const auto& dt = isnan(system[0].timestamp) ? 0.0 : sensor.timestamp - system[0].timestamp;

            // slam.frames[1].pose_gt = pose_gt_of_imu0_in_world * calibration.cameras[0].pose_in_imu0;
            // slam.frames[0].pose    = isnan(slam[0].timestamp) ? pose_gt_of_imu0_in_world * calibration.cameras[0].pose_in_imu0 : slam[0].pose;
            // slam.frames[1].pose    = motion.predict(slam[0].pose, dt);

            // PREPROCESS
            frame::processed processed = { };
            {
                time_this time_this { system.durations.preprocessing };

                processed = utils::pre_process(sensor, calibration, _options.slam, clahe);
            }

            // TODO: separate keyline and keypoints pipelines
            // TRACK
            frame::tracked tracked = { };
            {
                time_this time_this { system.durations.tracking };

                tracked = utils::track(system[0], processed, calibration, _options.slam);
            }

            auto pose_data_3d3d = pose_data { };
            auto pose_data_3d2d = pose_data { };

            // ESTIMATE
            frame::slam slam_frame = { };
            {
                time_this time_this { system.durations.estimation };

                try
                {
                    pose_data_3d3d = utils::estimate_pose_3d3d
                    (
                        system[0].points3d,
                        tracked.points3d,
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
                        system[0].points3d,
                        tracked.keypoints[0],
                        calibration.camera_matrix[0],
                        _options.slam.threshold_3d2d
                    );
                }
                catch (const std::runtime_error& error)
                {
                    SPDLOG_WARN("Unable to estimate pose because: {}", error.what());
                }
            }

            system.counts.correspondences_3d2d         = pose_data_3d2d.indices.size();
            system.counts.correspondences_3d3d         = pose_data_3d3d.indices.size();
            system.counts.correspondences_3d2d_inliers = pose_data_3d2d.inliers.size();
            system.counts.correspondences_3d3d_inliers = pose_data_3d3d.inliers.size();

            const auto err3d3d_mean = utils::mean(pose_data_3d3d.errors);
            const auto err3d2d_mean = utils::mean(pose_data_3d2d.errors);
            SPDLOG_INFO("3D-3D mean error: {:.4f} m", err3d3d_mean);
            SPDLOG_INFO("3D-2D mean error: {:.4f} px", err3d2d_mean);

            const auto& pose = pose_data_3d3d.inliers.size() > pose_data_3d2d.inliers.size() ? pose_data_3d3d.pose : pose_data_3d2d.pose;

            system[1] = frame::slam { tracked, system[0].pose * pose.inv()};

            SPDLOG_INFO("Estimated pose:   {}", system[1].pose);
            SPDLOG_INFO("Groundtruth pose: {}", system[1].pose_gt);

            system.points3d += system[1].pose * system[1].points3d;
            system.lines3d += system[1].pose * system[1].lines3d;

            system.points3d.buildIndex();

            system.counts.points = system.points3d.size();

            motion.update(system[0].pose, system[1].pose, dt);

            system.counts.print();
            system.durations.print();

            writer.write(system);

            on_frame(system);

            if (_stop_token.stop_requested())
            {
                break;
            }
        }
    }
}
