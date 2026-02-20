#include "zenslam/slam_thread.h"

#include <cmath>
#include <utility>

#include <spdlog/spdlog.h>

#include <vtk-9.3/vtkLogger.h>

#include "zenslam/calibration/calibration.h"
#include "zenslam/utils/estimator.h"
#include "zenslam/motion/gravity_estimator.h"
#include "zenslam/io/groundtruth.h"
#include "zenslam/motion/inertial_predictor.h"
#include "zenslam/motion/motion_predictor.h"
#include "zenslam/processor.h"
#include "zenslam/time_this.h"
#include "zenslam/tracking/tracker.h"
#include "zenslam/utils/utils.h"
#include "zenslam/utils/utils_slam.h"
#include "zenslam/utils/utils_std.h"
#include "zenslam/frame/durations.h"
#include "zenslam/frame/estimated.h"
#include "zenslam/frame/system.h"
#include "zenslam/frame/writer.h"

zenslam::slam_thread::slam_thread(options options) :
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


void zenslam::slam_thread::loop()
{
    const auto& calibration = calibration::parse(_options.folder->calibration_file, _options.folder->imu_calibration_file, _options.slam->stereo_rectify);

    frame::system system { };

    // Create tracker (owns descriptor matcher configured from options)
    processor               processor { _options.slam, calibration };
    tracker                 tracker { calibration, _options.slam, system };
    const estimator         estimator { calibration, _options.slam };
    motion_predictor        motion { };
    inertial_predictor      inertial { cv::Vec3d { 0.0, 9.81, 0.0 }, calibration.cameras[0].pose_in_imu0 };
    gravity_estimator       gravity_estimator { };

    auto ground_truth = groundtruth::read(_options.folder->groundtruth_file);
    auto writer       = frame::writer(_options.folder->output / "frame_data.csv");

    calibration.print();

    // Set configuration strings once
    system.counts.detector_type   = magic_enum::enum_name(_options.slam->feature.value());
    system.counts.descriptor_type = magic_enum::enum_name(_options.slam->descriptor.value());
    system.counts.matcher_type    = magic_enum::enum_name(_options.slam->matcher.value());

    while (!_stop_token.stop_requested())
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
            {
                pose_predicted = motion.predict(system[0], processed);

                pose_predicted = inertial.predict(system[0], processed);
            }

            // TRACK
            frame::tracked tracked = { };
            {
                time_this time_this { system.durations.tracking };

                tracked = tracker.track(system[0], processed, pose_predicted);

                // Update counts related to point features
                const auto& kp0_prev = system[0].keypoints[0];
                const auto& kp1_prev = system[0].keypoints[1];
                const auto& kp0_cur  = tracked.keypoints[0];
                const auto& kp1_cur  = tracked.keypoints[1];

                system.counts.points.features_l         = kp0_cur.size();
                system.counts.points.features_r         = kp1_cur.size();
                system.counts.points.features_l_tracked = kp0_cur.keys_matched(kp0_prev).size();
                system.counts.points.features_r_tracked = kp1_cur.keys_matched(kp1_prev).size();
                system.counts.points.features_l_new     = system.counts.points.features_l - system.counts.points.features_l_tracked;
                system.counts.points.features_r_new     = system.counts.points.features_r - system.counts.points.features_r_tracked;
                system.counts.points.features_total     = system.counts.points.features_l + system.counts.points.features_r;
                system.counts.points.matches_stereo     = kp0_cur.keys_matched(kp1_cur).size();
                system.counts.points.triangulated_3d    = tracked.points3d.size();

                // Update counts related to line features
                const auto& kl0_prev = system[0].keylines[0];
                const auto& kl1_prev = system[0].keylines[1];
                const auto& kl0_cur  = tracked.keylines[0];
                const auto& kl1_cur  = tracked.keylines[1];

                system.counts.lines.features_l         = kl0_cur.size();
                system.counts.lines.features_r         = kl1_cur.size();
                system.counts.lines.features_l_tracked = kl0_cur.keys_matched(kl0_prev).size();
                system.counts.lines.features_r_tracked = kl1_cur.keys_matched(kl1_prev).size();
                system.counts.lines.features_l_new     = system.counts.lines.features_l - system.counts.lines.features_l_tracked;
                system.counts.lines.features_r_new     = system.counts.lines.features_r - system.counts.lines.features_r_tracked;
                system.counts.lines.features_total     = system.counts.lines.features_l + system.counts.lines.features_r;
                system.counts.lines.matches_stereo     = kl0_cur.keys_matched(kl1_cur).size();
                system.counts.lines.triangulated_3d    = tracked.lines3d.size();

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
                    system.counts.klt_success_rate = static_cast<double>(system.counts.points.features_l_tracked) / static_cast<double>(kp0_prev.size());
                    system.counts.klt_success_rate = static_cast<double>(system.counts.points.features_l_tracked) / static_cast<double>(kp0_prev.size());
                }
                else
                {
                    system.counts.klt_success_rate = std::nan("nan");
                }
            }

            // ESTIMATE
            frame::estimated slam_frame = { };
            {
                time_this time_this { system.durations.estimation };

                const auto& estimate_result = estimator.estimate_pose_new(system[0], tracked);

                // Apply pose update (estimate is relative camera pose between frames)
                system[1] = frame::estimated { tracked, system[0].pose * estimate_result.chosen_pose.inv() };

                system.points3d += system[1].pose * system[1].points3d;
                system.lines3d  += system[1].pose * system[1].lines3d;

                // system.points3d.buildIndex();
                // system.lines3d.buildIndex();

                system.counts.map_points = system.points3d.size();
                system.counts.map_lines  = system.lines3d.size();

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
