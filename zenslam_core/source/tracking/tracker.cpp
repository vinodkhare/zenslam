#include "zenslam/tracking/tracker.h"

#include <iostream>
#include <thread>
#include <utility>

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
        _calibration(std::move(calib)),
        _options(std::move(opts)),
        _matcher(_options, _options.descriptor == descriptor_type::ORB || _options.descriptor == descriptor_type::FREAK),
        _detector(detector::create(_options)),
        _system(system)
    {
    }

    auto tracker::track
    (
        const frame::tracked&   frame_0,
        const frame::processed& frame_1
    ) const
        -> frame::tracked
    {
        map<keypoint> keypoints_0 = {};
        map<keypoint> keypoints_1 = {};
        map<keyline>  keylines_0  = {};
        map<keyline>  keylines_1  = {};
        map<point3d>  points3d    = {};
        map<line3d>   lines3d     = {};

        // Timing variables
        std::chrono::system_clock::duration time_stereo_tracking{0};
        std::chrono::system_clock::duration time_cross_tracking{0};
        std::chrono::system_clock::duration time_triangulation{0};
        std::chrono::system_clock::duration time_keyline_matching{0};
        std::chrono::system_clock::duration time_keyline_triangulation{0};
        std::chrono::system_clock::duration time_total{0};
        auto timer_total = time_this(time_total);

        {
            auto timer_stereo = time_this(time_stereo_tracking);
            std::jthread thread_0
            {
                [&]()
                {
                    // Track keypoints
                    const auto& keypoints_0_tracked = track_keypoints(frame_0.pyramids[0], frame_1.pyramids[0], frame_0.keypoints[0]);

                    keypoints_0.add(keypoints_0_tracked);

                    std::vector<keypoint> keypoints_0_detected{};
                    if (_options.detection.use_parallel_detector)
                    {
                        keypoints_0_detected = _detector.detect_keypoints_par(frame_1.undistorted[0], keypoints_0);
                    }
                    else
                    {
                        keypoints_0_detected = _detector.detect_keypoints(frame_1.undistorted[0], keypoints_0);
                    }

                    for (auto& keypoint : keypoints_0_detected)
                    {
                        auto point3d = _system.points3d.match(keypoint);

                        if (point3d.has_value())
                        {
                            keypoint.index = point3d.value().index;
                        }
                    }

                    keypoints_0.add(keypoints_0_detected);

                    // Track keylines
                    keylines_0 += track_keylines(frame_0.pyramids[0], frame_1.pyramids[0], frame_0.keylines[0]);

                    // Detect new keylines (avoiding areas where tracked keylines exist)
                    keylines_0 += _detector.detect_keylines(frame_1.undistorted[0], keylines_0, _options.triangulation.keyline_mask_margin);
                }
            };

            std::jthread thread_1
            {
                [&]()
                {
                    // Track keypoints
                    const auto& keypoints_1_tracked = track_keypoints(frame_0.pyramids[1], frame_1.pyramids[1], frame_0.keypoints[1]);
                    keypoints_1.add(keypoints_1_tracked);

                    std::vector<keypoint> keypoints_1_detected{};
                    if (_options.detection.use_parallel_detector)
                    {
                        keypoints_1_detected = _detector.detect_keypoints_par(frame_1.undistorted[1], keypoints_1);
                    }
                    else
                    {
                        keypoints_1_detected = _detector.detect_keypoints(frame_1.undistorted[1], keypoints_1);
                    }

                    for (auto& keypoint : keypoints_1_detected)
                    {
                        auto point3d = _system.points3d.match(keypoint);

                        if (point3d.has_value())
                        {
                            keypoint.index = point3d.value().index;
                        }
                    }

                    keypoints_1.add(keypoints_1_detected);

                    // Track keylines
                    keylines_1 += track_keylines(frame_0.pyramids[1], frame_1.pyramids[1], frame_0.keylines[1]);

                    // Detect new keylines (avoiding areas where tracked keylines exist)
                    keylines_1 += _detector.detect_keylines(frame_1.undistorted[1], keylines_1, _options.triangulation.keyline_mask_margin);
                }
            };
        }

        // Cross-camera (stereo) keypoint tracking
        {
            auto timer_cross = time_this(time_cross_tracking);
            // find keypoints_0 which are not in keypoints_1
            map<keypoint> keypoints_0_untracked{};
            for (const auto& [index, keypoint_0] : keypoints_0)
            {
                if (!keypoints_1.contains(index))
                {
                    keypoints_0_untracked.emplace(index, keypoint_0);
                }
            }

            auto keypoints_1_tracked = track_keypoints(frame_1.pyramids[0], frame_1.pyramids[1], keypoints_0_untracked);
            keypoints_1.add(keypoints_1_tracked);

            map<keypoint> keypoints_1_untracked{};
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

        // Triangulate keypoints
        {
            auto timer_tri = time_this(time_triangulation);
            // Match keypoints
            // keypoints_1 *= _matcher.match_keypoints(keypoints_0, keypoints_1);
            const auto& triangulated = _triangulator.triangulate_keypoints(keypoints_0, keypoints_1, frame_1.undistorted[0]);
            points3d.add(triangulated);
        }

        // Match keylines using the fundamental matrix
        {
            auto timer_kl_match = time_this(time_keyline_matching);
            const auto keyline_matches = utils::match_keylines
            (
                keylines_0,
                keylines_1,
                _calibration.fundamental_matrix[0],
                _options.epipolar_threshold
            );

            // Update keylines with matches (remap indices)
            keylines_1 *= keyline_matches;
        }

        // Triangulate keylines
        {
            auto timer_kl_tri = time_this(time_keyline_triangulation);
            lines3d += utils::triangulate_keylines
            (
                keylines_0,
                keylines_1,
                _calibration.projection_matrix[0],
                _calibration.projection_matrix[1],
                _options,
                _calibration.cameras[1].pose_in_cam0.translation()
            );
        }

        // Log timing information
        auto ms = [](const auto& duration) 
        { 
            return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count(); 
        };
        std::cout << "[TIMING] track() - Total: " << ms(time_total) << "ms "
                  << "| Stereo: " << ms(time_stereo_tracking) << "ms "
                  << "| Cross: " << ms(time_cross_tracking) << "ms "
                  << "| Triangulation: " << ms(time_triangulation) << "ms "
                  << "| KL Match: " << ms(time_keyline_matching) << "ms "
                  << "| KL Tri: " << ms(time_keyline_triangulation) << "ms" << std::endl;

        return {frame_1, {keypoints_0, keypoints_1}, {keylines_0, keylines_1}, points3d, lines3d};
    }

    // Moved from utils::track_keypoints
    std::vector<keypoint> tracker::track_keypoints
    (
        const std::vector<cv::Mat>&     pyramid_0,
        const std::vector<cv::Mat>&     pyramid_1,
        const map<keypoint>&            keypoints_map_0,
        const std::vector<cv::Point2f>& points_1_predicted
    ) const
    {
        if (keypoints_map_0.empty())
        {
            return {};
        }

        const auto& keypoints_0 = keypoints_map_0.values() | std::ranges::to<std::vector>();

        const auto& points_0 = keypoints_map_0.values_sliced<cv::Point2f>
        (
            [](const keypoint& kp) { return kp.pt; }
        ) | std::ranges::to<std::vector>();

        auto               points_1 = points_1_predicted;
        std::vector<uchar> status{};
        std::vector<float> errors{};

        cv::calcOpticalFlowPyrLK
        (
            pyramid_0,
            pyramid_1,
            points_0,
            points_1,
            status,
            errors,
            _options.tracking.klt_window_size,
            _options.tracking.klt_max_level,
            cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 99, 0.001),
            cv::OPTFLOW_LK_GET_MIN_EIGENVALS
        );

        std::vector<cv::Point2f> points_0_back{};
        std::vector<uchar>       status_back{};
        cv::calcOpticalFlowPyrLK
        (
            pyramid_1,
            pyramid_0,
            points_1,
            points_0_back,
            status_back,
            errors,
            _options.tracking.klt_window_size,
            _options.tracking.klt_max_level,
            cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 99, 0.001),
            cv::OPTFLOW_LK_GET_MIN_EIGENVALS
        );

        assert(points_0.size() == points_1.size() && points_1.size() == status.size() && status.size() == errors.size());

        std::vector<cv::Point2f> candidate_points_0;
        std::vector<cv::Point2f> candidate_points_1;
        std::vector<size_t>      candidate_indices;

        for (size_t i = 0; i < points_1.size(); ++i)
        {
            if
            (
                status[i] && status_back[i] &&
                cv::norm(points_0_back[i] - points_0[i]) < _options.tracking.klt_threshold
            )
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

    std::vector<keyline> tracker::track_keylines
    (
        const std::vector<cv::Mat>& pyramid_0,
        const std::vector<cv::Mat>& pyramid_1,
        const map<keyline>&         keylines_map_0
    ) const
    {
        return utils::track_keylines(pyramid_0, pyramid_1, keylines_map_0, _options);
    }
}
