#include "zenslam/tracking/tracker.h"

#include <algorithm>
#include <iostream>
#include <ranges>
#include <thread>
#include <utility>

#include <opencv2/features2d.hpp>
#include <opencv2/video/tracking.hpp>

#include "zenslam/detection/detector.h"
#include "zenslam/mapping/triangulation_utils.h"
#include "zenslam/mapping/triangulator.h"
#include "zenslam/matching/matcher.h"
#include "zenslam/matching/matching_utils.h"
#include "zenslam/time_this.h"
#include "zenslam/tracking/tracking_utils.h"

namespace zenslam
{
    tracker::tracker(calibration calib, slam_options opts, frame::system& system) :
        _calibration(std::move(calib)), _tracking(opts.tracking), _detection(opts.detection), _triangulation(opts.triangulation), _epipolar_threshold(opts.epipolar_threshold),
        _matcher(opts, opts.detection.descriptor == descriptor_type::ORB || opts.detection.descriptor == descriptor_type::FREAK), _triangulator(_calibration, opts), _detector(detector::create(opts.detection)),
        _system(system)
    {
    }

    auto tracker::assign_landmark_indices(
        std::vector<keypoint>& keypoints, const point3d_cloud& points3d, const cv::Point3d& camera_center, const double match_radius, const double max_descriptor_distance) const -> void
    {
        if (keypoints.empty() || points3d.empty())
        {
            return;
        }

        std::vector<point3d> nearby_points;
        if (match_radius > 0.0)
        {
            nearby_points = points3d.radius_search(static_cast<point3d>(camera_center), match_radius);
            if (nearby_points.empty())
            {
                return;
            }
        }

        std::vector<size_t>  landmark_indices;
        std::vector<cv::Mat> landmark_descriptors;
        const size_t         reserve_size = nearby_points.empty() ? points3d.size() : nearby_points.size();
        landmark_indices.reserve(reserve_size);
        landmark_descriptors.reserve(reserve_size);

        if (nearby_points.empty())
        {
            for (const auto& point3d : points3d | std::views::values)
            {
                if (point3d.descriptor.empty())
                {
                    continue;
                }

                landmark_indices.emplace_back(point3d.index);
                landmark_descriptors.emplace_back(point3d.descriptor);
            }
        }
        else
        {
            for (const auto& point3d : nearby_points)
            {
                if (point3d.descriptor.empty())
                {
                    continue;
                }

                landmark_indices.emplace_back(point3d.index);
                landmark_descriptors.emplace_back(point3d.descriptor);
            }
        }

        if (landmark_descriptors.empty())
        {
            return;
        }

        std::vector<size_t>  keypoint_rows;
        std::vector<cv::Mat> keypoint_descriptors;
        keypoint_rows.reserve(keypoints.size());
        keypoint_descriptors.reserve(keypoints.size());

        for (size_t i = 0; i < keypoints.size(); ++i)
        {
            if (keypoints[i].descriptor.empty())
            {
                continue;
            }

            keypoint_rows.emplace_back(i);
            keypoint_descriptors.emplace_back(keypoints[i].descriptor);
        }

        if (keypoint_descriptors.empty())
        {
            return;
        }

        cv::Mat descriptors2d;
        cv::Mat descriptors3d;
        cv::vconcat(keypoint_descriptors, descriptors2d);
        cv::vconcat(landmark_descriptors, descriptors3d);

        const bool is_binary = descriptors2d.depth() == CV_8U && descriptors3d.depth() == CV_8U;
        if (!is_binary)
        {
            if (descriptors2d.depth() != CV_32F)
            {
                descriptors2d.convertTo(descriptors2d, CV_32F);
            }
            if (descriptors3d.depth() != CV_32F)
            {
                descriptors3d.convertTo(descriptors3d, CV_32F);
            }
        }

        cv::BFMatcher           matcher(is_binary ? cv::NORM_HAMMING : cv::NORM_L2, false);
        std::vector<cv::DMatch> matches;
        matcher.match(descriptors2d, descriptors3d, matches);

        for (const auto& match : matches)
        {
            if (match.distance <= max_descriptor_distance)
            {
                auto& keypoint = keypoints[keypoint_rows[match.queryIdx]];
                keypoint.index = landmark_indices[match.trainIdx];
            }
        }
    }

    auto tracker::track(const frame::tracked& frame_0, const frame::processed& frame_1) const -> frame::tracked
    {
        map<keypoint>                       keypoints_0 = {};
        map<keypoint>                       keypoints_1 = {};
        map<keyline>                        keylines_0  = {};
        map<keyline>                        keylines_1  = {};
        map<point3d>                        points3d    = {};
        map<line3d>                         lines3d     = {};

        // Timing variables
        std::chrono::system_clock::duration time_stereo_tracking { 0 };
        std::chrono::system_clock::duration time_cross_tracking { 0 };
        std::chrono::system_clock::duration time_triangulation { 0 };
        std::chrono::system_clock::duration time_keyline_matching { 0 };
        std::chrono::system_clock::duration time_keyline_triangulation { 0 };
        std::chrono::system_clock::duration time_total { 0 };
        auto                                timer_total = time_this(time_total);

        std::vector<keypoint>               keypoints_0_detected_all {};
        std::vector<keypoint>               keypoints_1_detected_all {};
        size_t                              keypoints_0_tracked_count = 0;
        size_t                              keypoints_1_tracked_count = 0;

        {
            auto         timer_stereo = time_this(time_stereo_tracking);
            std::jthread thread_0 { [&]()
                                    {
                                        // Track keypoints
                                        const auto& keypoints_0_tracked                      = track_keypoints(frame_0.pyramids[0], frame_1.pyramids[0], frame_0.keypoints[0]);

                                        keypoints_0_tracked_count = keypoints_0_tracked.size();
                                        keypoints_0.add(keypoints_0_tracked);

                                        std::vector<keypoint> keypoints_0_detected {};
                                        if (_detection.use_parallel_detector)
                                        {
                                            keypoints_0_detected = _detector.detect_keypoints_par(frame_1.undistorted[0], keypoints_0);
                                        }
                                        else
                                        {
                                            keypoints_0_detected = _detector.detect_keypoints(frame_1.undistorted[0], keypoints_0);
                                        }

                                        assign_landmark_indices(
                                            keypoints_0_detected, _system.points3d, _system[0].pose.translation(), _tracking.landmark_match_radius, _tracking.landmark_match_distance);

                                        keypoints_0_detected_all = keypoints_0_detected;
                                        keypoints_0.add(keypoints_0_detected);

                                        // Track keylines
                                        keylines_0 += track_keylines(frame_0.pyramids[0], frame_1.pyramids[0], frame_0.keylines[0]);

                                        // Detect new keylines (avoiding areas where tracked keylines exist)
                                        keylines_0 += _detector.detect_keylines(frame_1.undistorted[0], keylines_0, _triangulation.keyline_mask_margin);
                                    } };

            std::jthread thread_1 { [&]()
                                    {
                                        // Track keypoints
                                        const auto& keypoints_1_tracked                      = track_keypoints(frame_0.pyramids[1], frame_1.pyramids[1], frame_0.keypoints[1]);
                                        keypoints_1_tracked_count = keypoints_1_tracked.size();
                                        keypoints_1.add(keypoints_1_tracked);

                                        std::vector<keypoint> keypoints_1_detected {};
                                        if (_detection.use_parallel_detector)
                                        {
                                            keypoints_1_detected = _detector.detect_keypoints_par(frame_1.undistorted[1], keypoints_1);
                                        }
                                        else
                                        {
                                            keypoints_1_detected = _detector.detect_keypoints(frame_1.undistorted[1], keypoints_1);
                                        }

                                        assign_landmark_indices(
                                            keypoints_1_detected, _system.points3d, _system[0].pose.translation(), _tracking.landmark_match_radius, _tracking.landmark_match_distance);

                                        keypoints_1_detected_all = keypoints_1_detected;
                                        keypoints_1.add(keypoints_1_detected);

                                        // Track keylines
                                        keylines_1 += track_keylines(frame_0.pyramids[1], frame_1.pyramids[1], frame_0.keylines[1]);

                                        // Detect new keylines (avoiding areas where tracked keylines exist)
                                        keylines_1 += _detector.detect_keylines(frame_1.undistorted[1], keylines_1, _triangulation.keyline_mask_margin);
                                    } };
        }

        // Cross-camera (stereo) keypoint tracking
        {
            auto          timer_cross = time_this(time_cross_tracking);
            // find keypoints_0 which are not in keypoints_1
            map<keypoint> keypoints_0_untracked {};
            for (const auto& [index, keypoint_0] : keypoints_0)
            {
                if (!keypoints_1.contains(index))
                {
                    keypoints_0_untracked.emplace(index, keypoint_0);
                }
            }

            auto keypoints_1_tracked = track_keypoints(frame_1.pyramids[0], frame_1.pyramids[1], keypoints_0_untracked);
            keypoints_1.add(keypoints_1_tracked);

            map<keypoint> keypoints_1_untracked {};
            for (const auto& [index, keypoint_1] : keypoints_1)
            {
                if (!keypoints_0.contains(index))
                {
                    keypoints_1_untracked.emplace(index, keypoint_1);
                }
            }

            auto keypoints_0_tracked = track_keypoints(frame_1.pyramids[1], frame_1.pyramids[0], keypoints_1_untracked);
            keypoints_0.add(keypoints_0_tracked);
        }

        // Descriptor matching for new keypoints when KLT tracking is weak
        {
            const auto   prev_left     = frame_0.keypoints[0].size();
            const auto   prev_right    = frame_0.keypoints[1].size();
            const double ratio_left    = prev_left > 0 ? static_cast<double>(keypoints_0_tracked_count) / static_cast<double>(prev_left) : 1.0;
            const double ratio_right   = prev_right > 0 ? static_cast<double>(keypoints_1_tracked_count) / static_cast<double>(prev_right) : 1.0;
            const double tracked_ratio = std::min(ratio_left, ratio_right);

            if (tracked_ratio < _tracking.klt_min_tracked_ratio)
            {
                const auto  prev_keypoints_left  = frame_0.keypoints[0].values() | std::ranges::to<std::vector>();
                const auto  prev_keypoints_right = frame_0.keypoints[1].values() | std::ranges::to<std::vector>();
                const auto  matches_left         = _matcher.match_keypoints(prev_keypoints_left, keypoints_0_detected_all);
                const auto  matches_right        = _matcher.match_keypoints(prev_keypoints_right, keypoints_1_detected_all);
                
                std::vector<cv::DMatch> filtered_left;
                filtered_left.reserve(matches_left.size());
                for (const auto& match : matches_left)
                {
                    if (!keypoints_0.contains(match.queryIdx))
                    {
                        filtered_left.emplace_back(match);
                    }
                }

                std::vector<cv::DMatch> filtered_right;
                filtered_right.reserve(matches_right.size());
                for (const auto& match : matches_right)
                {
                    if (!keypoints_1.contains(match.queryIdx))
                    {
                        filtered_right.emplace_back(match);
                    }
                }

                keypoints_0 *= filtered_left;
                keypoints_1 *= filtered_right;
            }
        }

        // Triangulate keypoints
        {
            auto        timer_tri    = time_this(time_triangulation);
            // Match keypoints
            // keypoints_1 *= _matcher.match_keypoints(keypoints_0, keypoints_1);
            const auto& triangulated = _triangulator.triangulate_keypoints(keypoints_0, keypoints_1, frame_1.undistorted[0]);
            points3d.add(triangulated);
        }

        // Match keylines using the fundamental matrix
        {
            auto       timer_kl_match   = time_this(time_keyline_matching);
            const auto keyline_matches  = utils::match_keylines(keylines_0, keylines_1, _calibration.fundamental_matrix[0], _epipolar_threshold);

            // Update keylines with matches (remap indices)
            keylines_1                 *= keyline_matches;
        }

        // Triangulate keylines
        {
            slam_options options;
            options.triangulation  = _triangulation;

            auto timer_kl_tri      = time_this(time_keyline_triangulation);
            lines3d               += utils::triangulate_keylines(
                keylines_0, keylines_1, _calibration.projection_matrix[0], _calibration.projection_matrix[1], options, _calibration.cameras[1].pose_in_cam0.translation());
        }

        // Log timing information
        auto ms = [](const auto& duration) { return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count(); };
        std::cout << "[TIMING] track() - Total: " << ms(time_total) << "ms "
                  << "| Stereo: " << ms(time_stereo_tracking) << "ms "
                  << "| Cross: " << ms(time_cross_tracking) << "ms "
                  << "| Triangulation: " << ms(time_triangulation) << "ms "
                  << "| KL Match: " << ms(time_keyline_matching) << "ms "
                  << "| KL Tri: " << ms(time_keyline_triangulation) << "ms" << std::endl;

        return {
            frame_1, { keypoints_0, keypoints_1 },
             { keylines_0,  keylines_1  },
             points3d, lines3d
        };
    }

    // Moved from utils::track_keypoints
    std::vector<keypoint> tracker::track_keypoints(
        const std::vector<cv::Mat>& pyramid_0, const std::vector<cv::Mat>& pyramid_1, const map<keypoint>& keypoints_map_0, const std::vector<cv::Point2f>& points_1_predicted) const
    {
        if (keypoints_map_0.empty())
        {
            return {};
        }

        const auto&        keypoints_0 = keypoints_map_0.values() | std::ranges::to<std::vector>();
        const auto&        points_0    = keypoints_map_0.values_sliced<cv::Point2f>([](const keypoint& kp) { return kp.pt; }) | std::ranges::to<std::vector>();

        auto               points_1    = points_1_predicted;
        std::vector<uchar> status {};
        std::vector<float> errors {};

        cv::calcOpticalFlowPyrLK(
            pyramid_0,
            pyramid_1,
            points_0,
            points_1,
            status,
            errors,
            _tracking.klt_window_size,
            _tracking.klt_max_level,
            cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 99, 0.001),
            cv::OPTFLOW_LK_GET_MIN_EIGENVALS);

        std::vector<cv::Point2f> points_0_back {};
        std::vector<uchar>       status_back {};
        cv::calcOpticalFlowPyrLK(
            pyramid_1,
            pyramid_0,
            points_1,
            points_0_back,
            status_back,
            errors,
            _tracking.klt_window_size,
            _tracking.klt_max_level,
            cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 99, 0.001),
            cv::OPTFLOW_LK_GET_MIN_EIGENVALS);

        assert(points_0.size() == points_1.size() && points_1.size() == status.size() && status.size() == errors.size());

        std::vector<cv::Point2f> candidate_points_0;
        std::vector<cv::Point2f> candidate_points_1;
        std::vector<size_t>      candidate_indices;

        for (size_t i = 0; i < points_1.size(); ++i)
        {
            if (status[i] && status_back[i] && cv::norm(points_0_back[i] - points_0[i]) < _tracking.klt_threshold)
            {
                candidate_points_0.push_back(points_0[i]);
                candidate_points_1.push_back(points_1[i]);
                candidate_indices.push_back(i);
            }
        }

        std::vector<keypoint> tracked_keypoints;
        for (auto i : candidate_indices)
        {
            auto tracked_keypoint = keypoints_0[i];
            tracked_keypoint.pt   = points_1[i];
            tracked_keypoints.emplace_back(tracked_keypoint);
        }

        return tracked_keypoints;
    }

    std::vector<keyline> tracker::track_keylines(const std::vector<cv::Mat>& pyramid_0, const std::vector<cv::Mat>& pyramid_1, const map<keyline>& keylines_map_0) const
    {
        slam_options options;
        options.tracking = _tracking;
        return utils::track_keylines(pyramid_0, pyramid_1, keylines_map_0, options);
    }
} // namespace zenslam
