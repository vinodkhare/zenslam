#include "zenslam/tracking/keypoint_tracker.h"

#include <algorithm>
#include <ranges>

#include <opencv2/features2d.hpp>
#include <opencv2/video/tracking.hpp>

#include "zenslam/detection/detector.h"
#include "zenslam/matching/matcher.h"

namespace zenslam
{
    keypoint_tracker::keypoint_tracker(calibration calib, slam_options opts, frame::system& system) :
        _calibration(std::move(calib)),
        _tracking(opts.tracking),
        _detection(opts.detection),
        _triangulation(opts.triangulation),
        _matcher(opts, opts.detection.descriptor == descriptor_type::ORB || opts.detection.descriptor == descriptor_type::FREAK),
        _triangulator(_calibration, opts),
        _detector(detector::create(opts.detection)),
        _system(system)
    {
    }

    auto keypoint_tracker::track(const frame::tracked& frame_0, const frame::processed& frame_1) const -> std::array<map<keypoint>, 2>
    {
        map<keypoint> keypoints_0 = { };
        map<keypoint> keypoints_1 = { };

        // Track keypoints
        const auto& keypoints_0_tracked = track_keypoints(frame_0.pyramids[0], frame_1.pyramids[0], frame_0.keypoints[0]);
        keypoints_0.add(keypoints_0_tracked);

        std::vector<keypoint> keypoints_0_detected { };
        if (_detection.use_parallel_detector)
        {
            keypoints_0_detected = _detector.detect_keypoints_par(frame_1.undistorted[0], keypoints_0);
        }
        else
        {
            keypoints_0_detected = _detector.detect_keypoints(frame_1.undistorted[0], keypoints_0);
        }

        assign_landmark_indices
        (
            keypoints_0_detected,
            _system.points3d,
            _system[0].pose.translation(),
            _tracking.landmark_match_radius,
            _tracking.landmark_match_distance
        );

        keypoints_0.add(keypoints_0_detected);

        const auto& keypoints_1_tracked = track_keypoints(frame_0.pyramids[1], frame_1.pyramids[1], frame_0.keypoints[1]);
        keypoints_1.add(keypoints_1_tracked);

        std::vector<keypoint> keypoints_1_detected { };
        if (_detection.use_parallel_detector)
        {
            keypoints_1_detected = _detector.detect_keypoints_par(frame_1.undistorted[1], keypoints_1);
        }
        else
        {
            keypoints_1_detected = _detector.detect_keypoints(frame_1.undistorted[1], keypoints_1);
        }

        keypoint_tracker::assign_landmark_indices
        (
            keypoints_1_detected,
            _system.points3d,
            _system[0].pose.translation(),
            _tracking.landmark_match_radius,
            _tracking.landmark_match_distance
        );

        keypoints_1.add(keypoints_1_detected);

        // Cross-camera (stereo) keypoint tracking
        {
            // find keypoints_0 which are not in keypoints_1
            map<keypoint> keypoints_0_untracked { };
            for (const auto& [index, keypoint_0] : keypoints_0) { if (!keypoints_1.contains(index)) { keypoints_0_untracked.emplace(index, keypoint_0); } }

            auto keypoints_1_tracked_new = track_keypoints(frame_1.pyramids[0], frame_1.pyramids[1], keypoints_0_untracked);
            keypoints_1.add(keypoints_1_tracked_new);

            map<keypoint> keypoints_1_untracked { };
            for (const auto& [index, keypoint_1] : keypoints_1) { if (!keypoints_0.contains(index)) { keypoints_1_untracked.emplace(index, keypoint_1); } }

            auto keypoints_0_tracked_new = track_keypoints(frame_1.pyramids[1], frame_1.pyramids[0], keypoints_1_untracked);
            keypoints_0.add(keypoints_0_tracked_new);
        }

        return { keypoints_0, keypoints_1 };
    }

    auto keypoint_tracker::triangulate(const std::array<map<keypoint>, 2>& keypoints, const cv::Mat& image) const -> point3d_cloud
    {
        return _triangulator.triangulate_keypoints(keypoints[0], keypoints[1], image);
    }

    std::vector<keypoint> keypoint_tracker::track_keypoints
    (
        const std::vector<cv::Mat>& pyramid_0,
        const std::vector<cv::Mat>& pyramid_1,
        const map<keypoint>&        keypoints_map_0
    ) const
    {
        if (keypoints_map_0.empty()) { return { }; }

        const auto& keypoints_0 = keypoints_map_0.values() | std::ranges::to<std::vector>();
        const auto& points_0    = keypoints_map_0.values_sliced<cv::Point2f>([](const keypoint& kp) { return kp.pt; }) | std::ranges::to<std::vector>();

        auto               points_1 = points_0;
        std::vector<uchar> status { };
        std::vector<float> errors { };

        cv::calcOpticalFlowPyrLK
        (
            pyramid_0,
            pyramid_1,
            points_0,
            points_1,
            status,
            errors,
            _tracking.klt_window_size,
            _tracking.klt_max_level,
            cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 99, 0.001),
            cv::OPTFLOW_LK_GET_MIN_EIGENVALS
        );

        std::vector<cv::Point2f> points_0_back { };
        std::vector<uchar>       status_back { };
        cv::calcOpticalFlowPyrLK
        (
            pyramid_1,
            pyramid_0,
            points_1,
            points_0_back,
            status_back,
            errors,
            _tracking.klt_window_size,
            _tracking.klt_max_level,
            cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 99, 0.001),
            cv::OPTFLOW_LK_GET_MIN_EIGENVALS
        );

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

    auto keypoint_tracker::assign_landmark_indices
    (
        std::vector<keypoint>& keypoints,
        const point3d_cloud&   points3d,
        const cv::Point3d&     camera_center,
        const double           match_radius,
        const double           max_descriptor_distance
    ) -> void
    {
        if (keypoints.empty() || points3d.empty()) { return; }

        std::vector<point3d> nearby_points;
        if (match_radius > 0.0)
        {
            nearby_points = points3d.radius_search(static_cast<point3d>(camera_center), match_radius);
            if (nearby_points.empty()) { return; }
        }

        std::vector<size_t>  landmark_indices;
        std::vector<cv::Mat> landmark_descriptors;
        const auto           reserve_size = nearby_points.empty() ? points3d.size() : nearby_points.size();
        landmark_indices.reserve(reserve_size);
        landmark_descriptors.reserve(reserve_size);

        if (nearby_points.empty())
        {
            for (const auto& point3d : points3d | std::views::values)
            {
                if (point3d.descriptor.empty()) { continue; }

                landmark_indices.emplace_back(point3d.index);
                landmark_descriptors.emplace_back(point3d.descriptor);
            }
        }
        else
        {
            for (const auto& point3d : nearby_points)
            {
                if (point3d.descriptor.empty()) { continue; }

                landmark_indices.emplace_back(point3d.index);
                landmark_descriptors.emplace_back(point3d.descriptor);
            }
        }

        if (landmark_descriptors.empty()) { return; }

        std::vector<size_t>  keypoint_rows;
        std::vector<cv::Mat> keypoint_descriptors;
        keypoint_rows.reserve(keypoints.size());
        keypoint_descriptors.reserve(keypoints.size());

        for (size_t i = 0; i < keypoints.size(); ++i)
        {
            if (keypoints[i].descriptor.empty()) { continue; }

            keypoint_rows.emplace_back(i);
            keypoint_descriptors.emplace_back(keypoints[i].descriptor);
        }

        if (keypoint_descriptors.empty()) { return; }

        cv::Mat descriptors2d;
        cv::Mat descriptors3d;
        cv::vconcat(keypoint_descriptors, descriptors2d);
        cv::vconcat(landmark_descriptors, descriptors3d);

        const auto is_binary = descriptors2d.depth() == CV_8U && descriptors3d.depth() == CV_8U;
        if (!is_binary)
        {
            if (descriptors2d.depth() != CV_32F) { descriptors2d.convertTo(descriptors2d, CV_32F); }
            if (descriptors3d.depth() != CV_32F) { descriptors3d.convertTo(descriptors3d, CV_32F); }
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
} // namespace zenslam
