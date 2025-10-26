#include "zenslam/tracker.h"

#include <thread>
#include <utility>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/video/tracking.hpp>

#include "zenslam/detector.h"
#include "zenslam/utils_slam.h"

namespace zenslam
{
    tracker::tracker(calibration calib, class options::slam opts)
        : _calibration(std::move(calib)), _options(std::move(opts))
    {
        const bool is_binary = _options.descriptor == descriptor_type::ORB || _options.descriptor == descriptor_type::FREAK;
        _matcher             = utils::create_matcher(_options, is_binary);
        _detector            = detector::create(_options);
    }

    auto tracker::track(const frame::tracked& frame_0, const frame::processed& frame_1) const -> frame::tracked
    {
        map<keypoint> keypoints_0 = { };
        map<keypoint> keypoints_1 = { };
        map<point3d>  points3d    = { };

        {
            std::jthread thread_0
            {
                [&]()
                {
                    keypoints_0 += track_keypoints(frame_0.pyramids[0], frame_1.pyramids[0], frame_0.keypoints[0]);

                    if (_options.use_parallel_detector)
                    {
                        keypoints_0 += _detector.detect_keypoints_par(frame_1.undistorted[0], keypoints_0);
                    }
                    else
                    {
                        keypoints_0 += _detector.detect_keypoints(frame_1.undistorted[0], keypoints_0);
                    }
                }
            };

            std::jthread thread_1
            {
                [&]()
                {
                    keypoints_1 += track_keypoints(frame_0.pyramids[1], frame_1.pyramids[1], frame_0.keypoints[1]);

                    if (_options.use_parallel_detector)
                    {
                        keypoints_1 += _detector.detect_keypoints_par(frame_1.undistorted[1], keypoints_1);
                    }
                    else
                    {
                        keypoints_1 += _detector.detect_keypoints(frame_1.undistorted[1], keypoints_1);
                    }
                }
            };
        }

        keypoints_1 *= utils::match_keypoints(keypoints_0, keypoints_1, _matcher, _options);

        points3d += utils::triangulate_keypoints
        (
            keypoints_0,
            keypoints_1,
            _calibration.projection_matrix[0],
            _calibration.projection_matrix[1],
            _options.triangulation_reprojection_threshold,
            _calibration.cameras[1].pose_in_cam0.translation()
        );

        return { frame_1, keypoints_0, keypoints_1, { }, points3d };
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
        const auto& keypoints_0 = keypoints_map_0.values() | std::ranges::to<std::vector>();
        if (keypoints_0.empty())
        {
            return { };
        }

        const auto& points_0 = keypoints_map_0.values_sliced<cv::Point2f>
        (
            [](const keypoint& kp) { return kp.pt; }
        ) | std::ranges::to<std::vector>();

        auto               points_1 = points_1_predicted;
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
            _options.klt_window_size,
            _options.klt_max_level,
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
            _options.klt_window_size,
            _options.klt_max_level,
            cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 99, 0.001),
            cv::OPTFLOW_LK_GET_MIN_EIGENVALS
        );

        assert(points_0.size() == points_1.size() && points_1.size() == status.size() && status.size() == errors.size());

        std::vector<cv::Point2f> candidate_points_0;
        std::vector<cv::Point2f> candidate_points_1;
        std::vector<size_t>      candidate_indices;

        for (size_t i = 0; i < points_1.size(); ++i)
        {
            if (status[i] && status_back[i] &&
                cv::norm(points_0_back[i] - points_0[i]) < _options.klt_threshold)
            {
                candidate_points_0.push_back(points_0[i]);
                candidate_points_1.push_back(points_1[i]);
                candidate_indices.push_back(i);
            }
        }

        std::vector<keypoint> tracked_keypoints;
        if (candidate_points_0.size() >= 8)
        {
            std::vector<uchar> inlier_mask { };
            cv::Mat            E { cv::findEssentialMat(candidate_points_0, candidate_points_1, _calibration.camera_matrix[0], cv::RANSAC, 0.999, _options.epipolar_threshold, inlier_mask) };

            for (size_t j = 0; j < candidate_indices.size(); ++j)
            {
                if (inlier_mask[j])
                {
                    auto i                = candidate_indices[j];
                    auto tracked_keypoint = keypoints_0[i];
                    tracked_keypoint.pt   = points_1[i];
                    tracked_keypoints.emplace_back(tracked_keypoint);
                }
            }
        }
        else
        {
            for (auto i : candidate_indices)
            {
                auto tracked_keypoint = keypoints_0[i];
                tracked_keypoint.pt   = points_1[i];
                tracked_keypoints.emplace_back(tracked_keypoint);
            }
        }

        return tracked_keypoints;
    }
}
