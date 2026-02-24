#include "zenslam/tracking/keypoint_tracker.h"

#include <algorithm>
#include <ranges>

#include <__thread/jthread.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/video/tracking.hpp>

#include "zenslam/detection/keypoint_detector_simple.h"
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
        _system(system)
    {
        switch (_detection.algorithm)
        {
        case detection_algorithm::SIMPLE:
            _detector = std::make_shared<keypoint_detector_simple>(_detection);
            break;
        case detection_algorithm::GRID:
            _detector = std::make_shared<keypoint_detector_grid>(_detection);
            break;
        case detection_algorithm::PARALLEL_GRID:
            _detector = std::make_shared<keypoint_detector_parallel>(_detection);
            break;
        }
    }

    auto keypoint_tracker::track(const frame::tracked& frame_0, const frame::processed& frame_1) const -> std::array<map<keypoint>, 2>
    {
        map<keypoint> keypoints_0 = { };
        map<keypoint> keypoints_1 = { };

        // Track keypoints
        const auto& keypoints_0_tracked = track_keypoints(frame_0.pyramids[0], frame_1.pyramids[0], frame_0.keypoints[0]);
        keypoints_0.add(keypoints_0_tracked);

        const auto& keypoints_1_tracked = track_keypoints(frame_0.pyramids[1], frame_1.pyramids[1], frame_0.keypoints[1]);
        keypoints_1.add(keypoints_1_tracked);

        const auto& keypoints_0_detected = _detector->detect_keypoints(frame_1.undistorted[0], keypoints_0);
        keypoints_0.add(keypoints_0_detected);

        const auto& keypoints_0_untracked
            = keypoints_0
            | std::views::values
            | std::views::filter([&keypoints_1](const auto& keypoint) { return !keypoints_1.contains(keypoint.index); })
            | std::ranges::to<std::vector>();

        const auto& keypoints_1_tracked_stereo = track_keypoints(frame_1.pyramids[0], frame_1.pyramids[1], keypoints_0_untracked);
        keypoints_1.add(keypoints_1_tracked_stereo);

        const auto& keypoints_1_detected = _detector->detect_keypoints(frame_1.undistorted[1], keypoints_1);
        keypoints_1.add(keypoints_1_detected);

        const auto& keypoints_1_untracked
            = keypoints_1
            | std::views::values
            | std::views::filter([&keypoints_0](const auto& keypoint) { return !keypoints_0.contains(keypoint.index); })
            | std::ranges::to<std::vector>();

        const auto& keypoints_0_tracked_stereo = track_keypoints(frame_1.pyramids[1], frame_1.pyramids[0], keypoints_1_untracked);
        keypoints_0.add(keypoints_0_tracked_stereo);

        map<keypoint> keypoints_0_filtered { };
        map<keypoint> keypoints_1_filtered { };

        if (_tracking.filter_epipolar)
        {
            const auto& filtered = filter_epipolar(keypoints_0, keypoints_1);

            for (const auto& [kp0, kp1] : filtered)
            {
                keypoints_0_filtered.add(kp0);
                keypoints_1_filtered.add(kp1);
            }
        }
        else
        {
            keypoints_0_filtered = keypoints_0;
            keypoints_1_filtered = keypoints_1;
        }

        return { keypoints_0_filtered, keypoints_1_filtered };
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

        const auto keypoints_0 = keypoints_map_0 | std::views::values | std::ranges::to<std::vector>();

        return track_keypoints(pyramid_0, pyramid_1, keypoints_0);
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

    auto keypoint_tracker::filter_epipolar(const map<keypoint>& keypoints_0, const map<keypoint>& keypoints_1) const -> std::vector<std::pair<keypoint, keypoint>>
    {
        const auto& keypoints_0_matched = keypoints_0.values_matched(keypoints_1) | std::ranges::to<std::vector>();
        const auto& keypoints_1_matched = keypoints_1.values_matched(keypoints_0) | std::ranges::to<std::vector>();

        const auto& point2f_0 = keypoints_0_matched | std::views::transform([](const keypoint& kp) { return kp.pt; }) | std::ranges::to<std::vector>();
        const auto& point2f_1 = keypoints_1_matched | std::views::transform([](const keypoint& kp) { return kp.pt; }) | std::ranges::to<std::vector>();

        const cv::Matx33d& fundamental_matrix = cv::findFundamentalMat
        (
            point2f_0,
            point2f_1,
            cv::FM_RANSAC,
            _triangulation.reprojection_threshold,
            0.99 // RANSAC confidence
        );

        const auto& error = std::views::zip(keypoints_0_matched, keypoints_1_matched) | std::views::transform
        (
            [&](const auto& pair)
            {
                const auto& [kp0, kp1] = pair;

                const auto pt0 = cv::Vec3d(kp0.pt.x, kp0.pt.y, 1);
                const auto pt1 = cv::Vec3d(kp1.pt.x, kp1.pt.y, 1);

                return (pt0.t() * fundamental_matrix * pt1)[0];
            }
        ) | std::ranges::to<std::vector>();

        return std::views::zip(keypoints_0_matched, keypoints_1_matched, error)
            | std::views::filter
            (
                [&](const auto& tuple)
                {
                    const auto& [kp0, kp1, err] = tuple;
                    return std::abs(err) < _tracking.epipolar_threshold; // Epipolar error threshold (tunable)
                }
            )
            | std::views::transform
            (
                [](const auto& tuple)
                {
                    const auto& [kp0, kp1, err] = tuple;
                    return std::make_pair(kp0, kp1);
                }
            )
            | std::ranges::to<std::vector>();
    }

    auto keypoint_tracker::track_keypoints
    (
        const std::vector<cv::Mat>&  pyramid_0,
        const std::vector<cv::Mat>&  pyramid_1,
        const std::vector<keypoint>& keypoints_0
    ) const -> std::vector<keypoint>
    {
        const auto& points_0 = keypoints_0 | std::views::transform([](const keypoint& kp) { return kp.pt; }) | std::ranges::to<std::vector>();

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
} // namespace zenslam
