#include "zenslam/slam_thread.h"

#include <utility>

#include <magic_enum/magic_enum.hpp>

#include <spdlog/spdlog.h>

#include <vtk-9.3/vtkLogger.h>

#include "zenslam/formatters.h"
#include "zenslam/processor.h"
#include "zenslam/time_this.h"
#include "zenslam/calibration/calibration.h"
#include "zenslam/frame/durations.h"
#include "zenslam/frame/estimated.h"
#include "zenslam/frame/system.h"
#include "zenslam/frame/writer.h"
#include "zenslam/io/groundtruth.h"
#include "zenslam/motion/gravity_estimator.h"
#include "zenslam/motion/inertial_predictor.h"
#include "zenslam/motion/motion_predictor.h"
#include "zenslam/tracking/tracker.h"
#include "zenslam/utils/estimator.h"
#include "zenslam/utils/utils.h"

zenslam::slam_thread::slam_thread(all_options options) :
    _options { std::move(options) } { vtkLogger::SetStderrVerbosity(vtkLogger::VERBOSITY_OFF); }


zenslam::slam_thread::~slam_thread()
{
    _stop_source.request_stop();
    // Wake the worker if it's waiting
    _cv.notify_all();
}

void zenslam::slam_thread::enqueue(const frame::sensor& frame)
{
    {
        std::lock_guard lock { _mutex };
        _queue.push(frame);
    }
    _cv.notify_one();
}

void zenslam::slam_thread::request_stop()
{
    _stop_source.request_stop();
    _cv.notify_all();
}

void zenslam::slam_thread::loop()
{
    const auto& calibration = calibration::parse
    (
        _options.folder.calibration_file,
        _options.folder.imu_calibration_file,
        _options.slam.detection.stereo_rectify,
        _options.slam.detection.rectify_alpha,
        _options.slam.detection.rectify_balance
    );

    frame::system system { };

    // Create tracker (owns descriptor matcher configured from options)
    processor          processor { _options.slam, calibration };
    tracker            tracker { calibration, _options.slam, system };
    const estimator    estimator { calibration, _options.slam };
    motion_predictor   motion { };
    inertial_predictor inertial { cv::Vec3d { 0.0, 9.81, 0.0 }, calibration.cameras[0].pose_in_imu0 };
    gravity_estimator  gravity_estimator { };
    bool               is_gravity_estimated { };

    auto ground_truth = groundtruth::read(_options.folder.groundtruth_file);
    auto writer       = frame::writer(_options.folder.output / "frame_data.csv");

    calibration.print();

    // Set configuration strings once
    system.counts.detector_type   = magic_enum::enum_name(_options.slam.detection.feature_detector);
    system.counts.descriptor_type = magic_enum::enum_name(_options.slam.detection.descriptor);
    system.counts.matcher_type    = magic_enum::enum_name(_options.slam.matcher);

    while (true)
    {
        {
            time_this t { system.durations.total };

            // READ FRAME
            frame::sensor sensor { };
            {
                time_this time_this { system.durations.wait };

                std::unique_lock lock { _mutex };
                _cv.wait(lock, [this] { return !_queue.empty() || _stop_token.stop_requested(); });

                if (_stop_token.stop_requested() && _queue.empty())
                {
                    break;
                }

                sensor = _queue.front();
                _queue.pop();
            }

            // PREPROCESS
            frame::processed processed { };
            {
                time_this time_this { system.durations.processing };

                processed = processor.process(sensor);
            }

            // PREDICT POSE
            cv::Affine3d pose_predicted { };
            cv::Affine3d pose_inertial { };
            {
                pose_predicted = motion.predict(system[0], processed);
                pose_inertial  = inertial.predict(system[0], processed);
            }

            // TRACK
            frame::tracked tracked = { };
            {
                time_this time_this { system.durations.tracking };

                tracked = tracker.track(system[0], processed, pose_inertial);

                const auto& matched_count_0 = tracked.keypoints[0].keys_matched(system[0].keypoints[0]).size();
                const auto& matched_count_1 = tracked.keypoints[1].keys_matched(system[0].keypoints[1]).size();
                const auto& matched_count   = tracked.keypoints[0].keys_matched(tracked.keypoints[1]).size();

                SPDLOG_INFO("#matched ( ->L): {}", matched_count_0);
                SPDLOG_INFO("#matched ( ->R): {}", matched_count_1);
                SPDLOG_INFO("#matched (L->R): {}", matched_count);
                SPDLOG_INFO("#triangulated: {}", tracked.points3d.size());
            }

            // ESTIMATE
            frame::estimated slam_frame = { };
            {
                time_this time_this { system.durations.estimation };

                estimate_pose_result estimate_result { };
                try
                {
                    estimate_result = estimator.estimate_pose_new(system[0], tracked);
                }
                catch (const std::exception& e)
                {
                    SPDLOG_ERROR("Pose estimation failed: {}", e.what());
                    estimate_result.chosen_pose  = pose_inertial;
                    estimate_result.chosen_count = 0;
                }

                SPDLOG_INFO("#pose predicted: {}", pose_predicted);
                SPDLOG_INFO("#pose estimated: {}", estimate_result.chosen_pose.inv());
                SPDLOG_INFO("#pose inertial:  {}", pose_inertial);

                // Apply pose update (estimate is relative camera pose between frames)
                system[1] = frame::estimated { tracked, system[0].pose * estimate_result.chosen_pose.inv() };

                system.points3d += system[1].pose * system[1].points3d;
                system.lines3d  += system[1].pose * system[1].lines3d;

                system.points3d.buildIndex();

                system.counts.map_points = system.points3d.size();
                system.counts.map_lines  = system.lines3d.size();

                writer.write(system);
            }

            // UPDATE MOTION MODEL
            {
                gravity_estimator.add(system[1], calibration.cameras[0].pose_in_imu0);

                if (gravity_estimator.has_estimate() && !is_gravity_estimated)
                {
                    auto g_estimated = gravity_estimator.estimate();
                    inertial.set_gravity_in_world(g_estimated);
                    SPDLOG_INFO("Estimated gravity vector in world frame: {}", g_estimated);
                    is_gravity_estimated = true;
                }

                motion.update(system[0], system[1]);
                inertial.update(system[0], system[1]);
            }

            on_frame(system);

            system[0] = std::move(system[1]);
        }

        //system.counts.print();
        //system.durations.print();
    }
}
