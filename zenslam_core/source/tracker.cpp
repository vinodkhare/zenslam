#include "zenslam/tracker.h"

#include <thread>
#include <utility>

#include <opencv2/calib3d.hpp>
#include <opencv2/video/tracking.hpp>

#include "zenslam/detector.h"
#include "zenslam/matcher.h"
#include "zenslam/tracking_utils.h"
#include "zenslam/triangulation_utils.h"
#include "zenslam/triangulator.h"
#include "zenslam/utils_opencv.h"
#include "zenslam/utils_slam.h"

namespace zenslam
{
    tracker::tracker(calibration calib, slam_options opts) :
        _calibration(std::move(calib)),
        _options(std::move(opts)),
        _matcher(_options, _options.descriptor == descriptor_type::ORB || _options.descriptor == descriptor_type::FREAK),
        _detector(detector::create(_options))
    {
    }

    auto tracker::track(const frame::tracked& frame_0, const frame::processed& frame_1) const -> frame::tracked
    {
        map<keypoint> keypoints_0 = { };
        map<keypoint> keypoints_1 = { };
        map<keyline>  keylines_0  = { };
        map<keyline>  keylines_1  = { };
        map<point3d>  points3d    = { };
        map<line3d>   lines3d     = { };

        {
            std::jthread thread_0
            {
                [&]()
                {
                    // Track keypoints
                    keypoints_0 += track_keypoints(frame_0.pyramids[0], frame_1.pyramids[0], frame_0.keypoints[0]);

                    if (_options.use_parallel_detector)
                    {
                        keypoints_0 += _detector.detect_keypoints_par(frame_1.undistorted[0], keypoints_0);
                    }
                    else
                    {
                        keypoints_0 += _detector.detect_keypoints(frame_1.undistorted[0], keypoints_0);
                    }

                    // Track keylines
                    keylines_0 += track_keylines(frame_0.pyramids[0], frame_1.pyramids[0], frame_0.keylines[0]);

                    // Detect new keylines (avoiding areas where tracked keylines exist)
                    keylines_0 += _detector.detect_keylines(frame_1.undistorted[0], keylines_0, _options.keyline_mask_margin);
                }
            };

            std::jthread thread_1
            {
                [&]()
                {
                    // Track keypoints
                    keypoints_1 += track_keypoints(frame_0.pyramids[1], frame_1.pyramids[1], frame_0.keypoints[1]);

                    if (_options.use_parallel_detector)
                    {
                        keypoints_1 += _detector.detect_keypoints_par(frame_1.undistorted[1], keypoints_1);
                    }
                    else
                    {
                        keypoints_1 += _detector.detect_keypoints(frame_1.undistorted[1], keypoints_1);
                    }

                    // Track keylines
                    keylines_1 += track_keylines(frame_0.pyramids[1], frame_1.pyramids[1], frame_0.keylines[1]);

                    // Detect new keylines (avoiding areas where tracked keylines exist)
                    keylines_1 += _detector.detect_keylines(frame_1.undistorted[1], keylines_1, _options.keyline_mask_margin);
                }
            };
        }

        // Match keypoints
        keypoints_1 *= _matcher.match_keypoints(keypoints_0, keypoints_1);
        points3d    += _triangulator.triangulate_keypoints(keypoints_0, keypoints_1);

        // Match keylines using the fundamental matrix
        const auto keyline_matches = utils::match_keylines
        (
            keylines_0,
            keylines_1,
            _calibration.fundamental_matrix[0],
            _options.epipolar_threshold
        );

        // Update keylines with matches (remap indices)
        keylines_1 *= keyline_matches;

        // Triangulate keylines
        lines3d += utils::triangulate_keylines
        (
            keylines_0,
            keylines_1,
            _calibration.projection_matrix[0],
            _calibration.projection_matrix[1],
            _options,
            _calibration.cameras[1].pose_in_cam0.translation()
        );

        return { frame_1, { keypoints_0, keypoints_1 }, { keylines_0, keylines_1 }, points3d, lines3d };
    }

    auto tracker::track(const frame::estimated& frame_0, const frame::processed& frame_1, const cv::Affine3d& pose_predicted) const -> frame::tracked
    {
        map<keypoint> keypoints_0 = { };
        map<keypoint> keypoints_1 = { };
        map<keyline>  keylines_0  = { };
        map<keyline>  keylines_1  = { };
        map<point3d>  points3d    = { };
        map<line3d>   lines3d     = { };

        // Compute relative camera-0 motion from previous to predicted current
        const cv::Affine3d T_01 = frame_0.pose * pose_predicted.inv();

        // Prepare predicted 2D positions for KLT initialization
        std::vector<cv::Point2f> preds_l;
        std::vector<cv::Point2f> preds_r;
        {
            const auto keypoints_prev_l = frame_0.keypoints[0].values() | std::ranges::to<std::vector>();
            preds_l.reserve(keypoints_prev_l.size());
            for (const auto& kp : keypoints_prev_l)
            {
                if (frame_0.points3d.contains(kp.index))
                {
                    const cv::Point3d X0 = frame_0.points3d.at(kp.index);
                    const cv::Point3d X1 = T_01 * X0; // now in current left-camera frame
                    const auto        uv = utils::project(std::vector<cv::Point3d> { X1 }, _calibration.projection_matrix[0]);
                    preds_l.emplace_back(cv::Point2f(static_cast<float>(uv[0].x), static_cast<float>(uv[0].y)));
                }
                else
                {
                    preds_l.emplace_back(kp.pt);
                }
            }

            const auto keypoints_prev_r = frame_0.keypoints[1].values() | std::ranges::to<std::vector>();
            preds_r.reserve(keypoints_prev_r.size());
            for (const auto& kp : keypoints_prev_r)
            {
                if (frame_0.points3d.contains(kp.index))
                {
                    const cv::Point3d X0 = frame_0.points3d.at(kp.index);
                    const cv::Point3d X1 = T_01 * X0; // current left-camera frame
                    // Project into current right camera using stereo projection matrix
                    const auto uv = utils::project(std::vector { X1 }, _calibration.projection_matrix[1]);
                    preds_r.emplace_back(cv::Point2f(static_cast<float>(uv[0].x), static_cast<float>(uv[0].y)));
                }
                else
                {
                    preds_r.emplace_back(kp.pt);
                }
            }
        }

        {
            std::jthread thread_0
            {
                [&]()
                {
                    // Track keypoints
                    keypoints_0 += track_keypoints(frame_0.pyramids[0], frame_1.pyramids[0], frame_0.keypoints[0], preds_l);

                    if (_options.use_parallel_detector)
                    {
                        keypoints_0 += _detector.detect_keypoints_par(frame_1.undistorted[0], keypoints_0);
                    }
                    else
                    {
                        keypoints_0 += _detector.detect_keypoints(frame_1.undistorted[0], keypoints_0);
                    }

                    // Track keylines
                    keylines_0 += track_keylines(frame_0.pyramids[0], frame_1.pyramids[0], frame_0.keylines[0]);

                    // Detect new keylines (avoiding areas where tracked keylines exist)
                    keylines_0 += _detector.detect_keylines(frame_1.undistorted[0], keylines_0, _options.keyline_mask_margin);
                }
            };

            std::jthread thread_1
            {
                [&]()
                {
                    // Track keypoints
                    keypoints_1 += track_keypoints(frame_0.pyramids[1], frame_1.pyramids[1], frame_0.keypoints[1], preds_r);

                    if (_options.use_parallel_detector)
                    {
                        keypoints_1 += _detector.detect_keypoints_par(frame_1.undistorted[1], keypoints_1);
                    }
                    else
                    {
                        keypoints_1 += _detector.detect_keypoints(frame_1.undistorted[1], keypoints_1);
                    }

                    // Track keylines
                    keylines_1 += track_keylines(frame_0.pyramids[1], frame_1.pyramids[1], frame_0.keylines[1]);

                    // Detect new keylines (avoiding areas where tracked keylines exist)
                    keylines_1 += _detector.detect_keylines(frame_1.undistorted[1], keylines_1, _options.keyline_mask_margin);
                }
            };
        }

        // Match keypoints
        keypoints_1 *= _matcher.match_keypoints(keypoints_0, keypoints_1);
        points3d    += _triangulator.triangulate_keypoints(keypoints_0, keypoints_1);

        // Match keylines using the fundamental matrix
        const auto keyline_matches = utils::match_keylines
        (
            keylines_0,
            keylines_1,
            _calibration.fundamental_matrix[0],
            _options.epipolar_threshold
        );

        // Update keylines with matches (remap indices)
        keylines_1 *= keyline_matches;

        // Triangulate keylines
        lines3d += utils::triangulate_keylines
        (
            keylines_0,
            keylines_1,
            _calibration.projection_matrix[0],
            _calibration.projection_matrix[1],
            _options,
            _calibration.cameras[1].pose_in_cam0.translation()
        );

        return { frame_1, { keypoints_0, keypoints_1 }, { keylines_0, keylines_1 }, points3d, lines3d };
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
            cv::Mat            E {
                cv::findEssentialMat
                (candidate_points_0, candidate_points_1, _calibration.camera_matrix[0], cv::RANSAC, 0.999, _options.epipolar_threshold, inlier_mask)
            };

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
