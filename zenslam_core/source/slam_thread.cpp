#include "zenslam/slam_thread.h"

#include <cmath>
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
#include "zenslam/keyframe_selector.h"
#include "zenslam/motion_predictor.h"
#include "zenslam/processor.h"
#include "zenslam/time_this.h"
#include "zenslam/tracker.h"
#include "zenslam/utils.h"
#include "zenslam/utils_slam.h"
#include "zenslam/utils_std.h"
#include "zenslam/formatters.h"

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
    const auto& calibration = calibration::parse(_options.folder->calibration_file, _options.folder->imu_calibration_file, _options.slam->stereo_rectify);

    // Create tracker (owns descriptor matcher configured from options)
    processor          processor{ _options.slam, calibration };
    tracker            tracker{ calibration, _options.slam };
    const estimator    estimator{ calibration, _options.slam };
    motion_predictor   motion{};
    inertial_predictor inertial{ cv::Vec3d{ 0.0, 9.81, 0.0 }, calibration.cameras[0].pose_in_imu0 };
    gravity_estimator  gravity_estimator{};
    keyframe_selector  keyframes{ _options.slam->keyframe };

    auto ground_truth = groundtruth::read(_options.folder->groundtruth_file);
    auto writer       = frame::writer(_options.folder->output / "frame_data.csv");

    calibration.print();

    frame::system system{};

    // Set configuration strings once
    system.counts.detector_type   = magic_enum::enum_name(_options.slam->feature.value());
    system.counts.descriptor_type = magic_enum::enum_name(_options.slam->descriptor.value());
    system.counts.matcher_type    = magic_enum::enum_name(_options.slam->matcher.value());

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

            // TRACK
            frame::tracked tracked = {};
            {
                time_this time_this{ system.durations.tracking };

                tracked = tracker.track(system[0], processed, pose_predicted);

                // Update counts related to point features
                const auto& kp0_prev = system[0].keypoints[0];
                const auto& kp1_prev = system[0].keypoints[1];
                const auto& kp0_cur  = tracked.keypoints[0];
                const auto& kp1_cur  = tracked.keypoints[1];

                system.counts.points.features_l          = kp0_cur.size();
                system.counts.points.features_r          = kp1_cur.size();
                system.counts.points.features_l_tracked  = kp0_cur.keys_matched(kp0_prev).size();
                system.counts.points.features_r_tracked  = kp1_cur.keys_matched(kp1_prev).size();
                system.counts.points.features_l_new      = system.counts.points.features_l - system.counts.points.features_l_tracked;
                system.counts.points.features_r_new      = system.counts.points.features_r - system.counts.points.features_r_tracked;
                system.counts.points.features_total      = system.counts.points.features_l + system.counts.points.features_r;
                system.counts.points.matches_stereo      = kp0_cur.keys_matched(kp1_cur).size();
                system.counts.points.triangulated_3d     = tracked.points3d.size();

                // Update counts related to line features
                const auto& kl0_prev = system[0].keylines[0];
                const auto& kl1_prev = system[0].keylines[1];
                const auto& kl0_cur  = tracked.keylines[0];
                const auto& kl1_cur  = tracked.keylines[1];

                system.counts.lines.features_l          = kl0_cur.size();
                system.counts.lines.features_r          = kl1_cur.size();
                system.counts.lines.features_l_tracked  = kl0_cur.keys_matched(kl0_prev).size();
                system.counts.lines.features_r_tracked  = kl1_cur.keys_matched(kl1_prev).size();
                system.counts.lines.features_l_new      = system.counts.lines.features_l - system.counts.lines.features_l_tracked;
                system.counts.lines.features_r_new      = system.counts.lines.features_r - system.counts.lines.features_r_tracked;
                system.counts.lines.features_total      = system.counts.lines.features_l + system.counts.lines.features_r;
                system.counts.lines.matches_stereo      = kl0_cur.keys_matched(kl1_cur).size();
                system.counts.lines.triangulated_3d     = tracked.lines3d.size();

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
            frame::estimated slam_frame = {};
            {
                time_this time_this{ system.durations.estimation };

                // Original estimation method - tries all 5 approaches
                auto estimate_result = estimator.estimate_pose(system[0], tracked);

                // Apply weighted fusion to combine all pose estimates
                auto weighted_result = estimator.estimate_pose_weighted(estimate_result);

                SPDLOG_INFO("\n========== POSE ESTIMATION RESULTS ==========");
                SPDLOG_INFO("Individual method results:");
                SPDLOG_INFO("  3D-3D Combined:   {} correspondences, {} inliers (points+lines unified)",
                    estimate_result.pose_3d3d.indices.size(), estimate_result.pose_3d3d.inliers.size());
                SPDLOG_INFO("  3D-2D Combined:   {} correspondences, {} inliers (points+lines unified)",
                    estimate_result.pose_3d2d.indices.size(), estimate_result.pose_3d2d.inliers.size());
                SPDLOG_INFO("  2D-2D Combined:   {} correspondences, {} inliers (points+lines unified)",
                    estimate_result.pose_2d2d.indices.size(), estimate_result.pose_2d2d.inliers.size());
                
                SPDLOG_INFO("Weighted fusion results:");
                SPDLOG_INFO("  Best method: {} ({} inliers)", 
                    weighted_result.best_method, weighted_result.best_method_inliers);
                SPDLOG_INFO("  Total inliers (all methods): {}", weighted_result.total_inliers);
                SPDLOG_INFO("  Overall confidence: {:.3f}", weighted_result.confidence);
                SPDLOG_INFO("  Weight breakdown:");
                if (weighted_result.weight_3d3d > 0.01)
                    SPDLOG_INFO("    3D-3D Combined:  {:.3f} (unified points+lines)", weighted_result.weight_3d3d);
                if (weighted_result.weight_3d2d > 0.01)
                    SPDLOG_INFO("    3D-2D Combined:  {:.3f} (unified points+lines)", weighted_result.weight_3d2d);
                if (weighted_result.weight_2d2d > 0.01)
                    SPDLOG_INFO("    2D-2D Combined:  {:.3f} (unified points+lines)", weighted_result.weight_2d2d);
                // Note: All _lines weights are now combined with corresponding point methods

                // Log covariance information
                SPDLOG_INFO("Pose uncertainty estimates:");
                if (weighted_result.has_valid_covariance)
                {
                    SPDLOG_INFO("  Translation std: {:.4f}m, Rotation std: {:.6f}rad ({:.4f}deg)",
                        weighted_result.translation_std, weighted_result.rotation_std,
                        weighted_result.rotation_std * 180.0 / M_PI);
                }
                else
                {
                    SPDLOG_WARN("  Covariance estimation failed (insufficient inliers for reliable uncertainty)");
                }

                // Update counts
                system.counts.correspondences_3d2d         = estimate_result.pose_3d2d.indices.size();
                system.counts.correspondences_3d3d         = estimate_result.pose_3d3d.indices.size();
                system.counts.correspondences_2d2d         = estimate_result.pose_2d2d.indices.size();
                system.counts.correspondences_3d2d_inliers = estimate_result.pose_3d2d.inliers.size();
                system.counts.correspondences_3d3d_inliers = estimate_result.pose_3d3d.inliers.size();
                system.counts.correspondences_2d2d_inliers = estimate_result.pose_2d2d.inliers.size();

                // Smart pose selection using confidence + covariance
                cv::Affine3d chosen_pose = weighted_result.pose;
                bool use_weighted = true;

                // Check if covariance is valid and translation uncertainty is reasonable
                if (!weighted_result.has_valid_covariance)
                {
                    SPDLOG_WARN("Covariance invalid (inliers < 5), cannot assess uncertainty reliably");
                    use_weighted = false;
                }
                else if (weighted_result.translation_std > 0.5)
                {
                    SPDLOG_WARN("Translation uncertainty too high ({:.3f}m > 0.5m), using motion prediction instead",
                        weighted_result.translation_std);
                    use_weighted = false;
                }
                else if (weighted_result.confidence < 0.15)
                {
                    SPDLOG_WARN("Confidence too low ({:.3f} < 0.15)", weighted_result.confidence);
                    use_weighted = false;
                }
                else if (weighted_result.total_inliers < 6)
                {
                    SPDLOG_WARN("Insufficient inliers ({} < 6)", weighted_result.total_inliers);
                    use_weighted = false;
                }

                // Fallback strategy: weighted → best single method → motion prediction
                if (!use_weighted)
                {
                    chosen_pose = estimate_result.chosen_pose;
                    
                    std::string method_name = "Unknown";
                    if (estimate_result.chosen_count == estimate_result.pose_3d3d.inliers.size())
                        method_name = "3D-3D Combined";
                    else if (estimate_result.chosen_count == estimate_result.pose_3d2d.inliers.size())
                        method_name = "3D-2D Combined";
                    else if (estimate_result.chosen_count == estimate_result.pose_2d2d.inliers.size())
                        method_name = "2D-2D Combined";
                    
                    SPDLOG_INFO("Falling back to best single method: {} ({} inliers)",
                        method_name, estimate_result.chosen_count);

                    // Final fallback: if single method also weak, use prediction
                    if (estimate_result.chosen_count < 5)
                    {
                        SPDLOG_WARN("Single method too weak (< 5 inliers), using motion prediction as last resort");
                        chosen_pose = pose_predicted;
                    }
                }
                else
                {
                    SPDLOG_INFO("Using weighted fusion pose (confidence={:.3f}, translation_std={:.4f}m)",
                        weighted_result.confidence, weighted_result.translation_std);
                }

                SPDLOG_INFO("================================================\n");

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

                system.counts.map_points = system.points3d.size();
                system.counts.map_lines  = system.lines3d.size();

                const auto decision = keyframes.decide(system[1], system.counts);
                system[1].is_keyframe = decision.is_keyframe;

                if (decision.is_keyframe)
                {
                    SPDLOG_INFO(
                        "Keyframe selected (frames_since_last={}, trans={:.3f}m, rot={:.2f}deg, tracked={:.2f}, inliers={}, forced={}, motion={}, quality={})",
                        decision.frames_since_last,
                        decision.translation,
                        decision.rotation_deg,
                        decision.tracked_ratio,
                        decision.inliers,
                        decision.forced,
                        decision.motion_triggered,
                        decision.quality_triggered
                    );
                }

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
