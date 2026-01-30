#include "zenslam/slam_thread.h"

#include <spdlog/spdlog.h>
#include <utility>
#include <vtk-9.3/vtkLogger.h>

#include "zenslam/calibration.h"
#include "zenslam/estimator.h"
#include "zenslam/frame/durations.h"
#include "zenslam/frame/estimated.h"
#include "zenslam/frame/system.h"
#include "zenslam/frame/writer.h"
#include "zenslam/gravity_estimator.h"
#include "zenslam/groundtruth.h"
#include "zenslam/inertial_predictor.h"
#include "zenslam/motion_predictor.h"
#include "zenslam/processor.h"
#include "zenslam/time_this.h"
#include "zenslam/tracker.h"
#include "zenslam/utils.h"
#include "zenslam/utils_slam.h"
#include "zenslam/utils_std.h"

zenslam::slam_thread::slam_thread(options options) : _options{ std::move(options) } { vtkLogger::SetStderrVerbosity(vtkLogger::VERBOSITY_OFF); }


zenslam::slam_thread::~slam_thread()
{
    _stop_source.request_stop();
    // Wake the worker if it's waiting
    _cv.notify_all();
}

void zenslam::slam_thread::enqueue(const frame::sensor& frame)
{
    {
        std::lock_guard lock{ _mutex };
        _queue.push(frame);
    }
    _cv.notify_one();
}


void zenslam::slam_thread::loop()
{
    const auto& calibration = calibration::parse(_options.folder.calibration_file, _options.folder.imu_calibration_file, _options.slam.stereo_rectify);

    // Create tracker (owns descriptor matcher configured from options)
    processor          processor{ _options.slam, calibration };
    tracker            tracker{ calibration, _options.slam };
    const estimator    estimator{ calibration, _options.slam };
    motion_predictor   motion{};
    inertial_predictor inertial{ cv::Vec3d{ 0.0, 9.81, 0.0 }, calibration.cameras[0].pose_in_imu0 };
    gravity_estimator  gravity_estimator{};

    auto ground_truth = groundtruth::read(_options.folder.groundtruth_file);
    auto writer       = frame::writer(_options.folder.output / "frame_data.csv");

    calibration.print();

    frame::system system{};

    // Set configuration strings once
    system.counts.detector_type   = magic_enum::enum_name(_options.slam.feature);
    system.counts.descriptor_type = magic_enum::enum_name(_options.slam.descriptor);
    system.counts.matcher_type    = magic_enum::enum_name(_options.slam.matcher);

    while (!_stop_token.stop_requested())
    {
        {
            time_this t{ system.durations.total };

            // READ FRAME
            frame::sensor sensor{};
            {
                time_this time_this{ system.durations.wait };

                SPDLOG_TRACE("Waiting for next frame");

                std::unique_lock lock{ _mutex };
                _cv.wait(lock, [this] { return !_queue.empty() || _stop_token.stop_requested(); });

                if (_stop_token.stop_requested() && _queue.empty())
                {
                    break;
                }

                sensor = _queue.front();
                _queue.pop();
            }

            // PREPROCESS
            frame::processed processed{};
            {
                time_this time_this{ system.durations.processing };

                processed = processor.process(sensor);
            }

            // PREDICT POSE
            cv::Affine3d pose_predicted{};
            {
                pose_predicted = motion.predict(system[0], processed);
                SPDLOG_INFO("Predicted pose (motion): {}", pose_predicted);

                pose_predicted = inertial.predict(system[0], processed);
                SPDLOG_INFO("Predicted pose (inertial): {}", pose_predicted);
            }

            // TODO: separate keyline and keypoints pipelines
            // TRACK
            frame::tracked tracked = {};
            {
                time_this time_this{ system.durations.tracking };

                tracked = tracker.track(system[0], processed, pose_predicted);

                // Update counts related to keypoints and matches
                const auto& kp0_prev = system[0].keypoints[0];
                const auto& kp1_prev = system[0].keypoints[1];
                const auto& kp0_cur  = tracked.keypoints[0];
                const auto& kp1_cur  = tracked.keypoints[1];

                system.counts.keypoints_l          = kp0_cur.size();
                system.counts.keypoints_r          = kp1_cur.size();
                system.counts.keypoints_l_tracked  = kp0_cur.keys_matched(kp0_prev).size();
                system.counts.keypoints_r_tracked  = kp1_cur.keys_matched(kp1_prev).size();
                system.counts.keypoints_l_new      = system.counts.keypoints_l - system.counts.keypoints_l_tracked;
                system.counts.keypoints_r_new      = system.counts.keypoints_r - system.counts.keypoints_r_tracked;
                system.counts.keypoints_total      = system.counts.keypoints_l + system.counts.keypoints_r;
                system.counts.matches              = kp0_cur.keys_matched(kp1_cur).size();
                system.counts.matches_triangulated = tracked.points3d.size();

                // Compute quality metrics
                // Response statistics (feature strength)
                auto responses_l = kp0_cur | std::views::transform([](const auto& pair) { return pair.second.response; }) | std::ranges::to<std::vector>();
                auto responses_r = kp1_cur | std::views::transform([](const auto& pair) { return pair.second.response; }) | std::ranges::to<std::vector>();

                system.counts.response_mean_l = utils::mean(responses_l);
                system.counts.response_mean_r = utils::mean(responses_r);
                system.counts.response_std_l  = utils::std_dev(responses_l);
                system.counts.response_std_r  = utils::std_dev(responses_r);

                // KLT success rate
                if (!kp0_prev.empty())
                {
                    system.counts.klt_success_rate = static_cast<double>(system.counts.keypoints_l_tracked) / static_cast<double>(kp0_prev.size());
                    system.counts.klt_success_rate = static_cast<double>(system.counts.keypoints_l_tracked) / static_cast<double>(kp0_prev.size());
                }
                else
                {
                    system.counts.klt_success_rate = std::nan("nan");
                }
            }

            // ESTIMATE
            frame::estimated slam_frame = {};
            {
                time_this time_this{ system.durations.estimation };

                auto [pose_3d3d, pose_3d2d, pose_2d2d, chosen_pose, chosen_count] = estimator.estimate_pose(system[0], tracked);

                SPDLOG_INFO("Chosen count: {}", chosen_count);

                // Update counts
                system.counts.correspondences_3d2d         = pose_3d2d.indices.size();
                system.counts.correspondences_3d3d         = pose_3d3d.indices.size();
                system.counts.correspondences_2d2d         = pose_2d2d.indices.size();
                system.counts.correspondences_3d2d_inliers = pose_3d2d.inliers.size();
                system.counts.correspondences_3d3d_inliers = pose_3d3d.inliers.size();
                system.counts.correspondences_2d2d_inliers = pose_2d2d.inliers.size();

                // Logging
                const auto err3d3d_mean = utils::mean(pose_3d3d.errors);
                const auto err3d2d_mean = utils::mean(pose_3d2d.errors);
                const auto err2d2d_mean = utils::mean(pose_2d2d.errors);

                SPDLOG_INFO("3D-3D mean error: {:.4f} m", err3d3d_mean);
                SPDLOG_INFO("3D-2D mean error: {:.4f} px", err3d2d_mean);
                SPDLOG_INFO("2D-2D mean error: {:.4f} px", err2d2d_mean);

                if (chosen_count < 10)
                {
                    chosen_pose = pose_predicted;
                }

                // Apply pose update (estimate is relative camera pose between frames)
                system[1] = frame::estimated{ tracked, system[0].pose * chosen_pose.inv() };

                // Gravity estimation using residual method (needs previous frame)
                if (gravity_estimator.count() < 50)
                {
                    gravity_estimator.add(system[1], calibration.cameras[0].pose_in_imu0);
                }
                else
                {
                    auto gravity = gravity_estimator.estimate();
                    inertial.set_gravity_in_world(gravity);
                    SPDLOG_INFO("Gravity estimated: [{:.3f}, {:.3f}, {:.3f}], mag: {:.3f}", gravity[0], gravity[1], gravity[2], cv::norm(gravity));
                }

                SPDLOG_INFO("Predicted pose:   {}", system[1].pose * pose_predicted.inv());
                SPDLOG_INFO("Estimated pose:   {}", system[1].pose);
                SPDLOG_INFO("Ground-truth pose: {}", system[1].pose_gt);

                system.points3d += system[1].pose * system[1].points3d;
                system.lines3d += system[1].pose * system[1].lines3d;

                // system.points3d.buildIndex();
                // system.lines3d.buildIndex();

                system.counts.points = system.points3d.size();
                system.counts.lines  = system.lines3d.size();

                writer.write(system);
            }

            // UPDATE MOTION MODEL
            {
                motion.update(system[0], system[1]);
                inertial.update(system[0], system[1]);
            }

            on_frame(system);

            system[0] = std::move(system[1]);
        }

        system.counts.print();
        system.durations.print();
    }
}
