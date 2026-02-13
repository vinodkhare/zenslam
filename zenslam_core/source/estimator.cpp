#include "zenslam/estimator.h"

#include <spdlog/spdlog.h>

#include "zenslam/pose_estimation/point_estimator.h"
#include "zenslam/pose_estimation/line_estimator.h"
#include "zenslam/pose_estimation/combined_estimator.h"
#include "zenslam/pose_estimation/pose_fusion.h"

namespace zenslam
{
    estimator::estimator(calibration calib, slam_options opts)
        : _calibration(std::move(calib))
        , _options(std::move(opts))
        , _point_estimator(std::make_unique<pose_estimation::point_estimator>(_calibration, _options))
        , _line_estimator(std::make_unique<pose_estimation::line_estimator>(_calibration, _options))
        , _combined_estimator(std::make_unique<pose_estimation::combined_estimator>(_calibration, _options))
    {
    }

    estimator::~estimator() = default;
    estimator::estimator(estimator&&) noexcept = default;
    estimator& estimator::operator=(estimator&&) noexcept = default;

    auto estimator::estimate_pose(
        const std::map<size_t, point3d>& points3d_0,
        const frame::tracked& tracked_1) const -> estimate_pose_result
    {
        estimate_pose_result result{};

        // Combined 3D-3D estimation using both points and lines
        if (auto pose = _combined_estimator->estimate_3d3d(
            points3d_0,
            tracked_1.points3d,
            std::map<size_t, line3d>{},  // No previous lines in this overload
            tracked_1.lines3d))
        {
            result.pose_3d3d = *pose;
            result.pose_3d3d_lines = *pose;  // Backward compatibility
        }

        // Combined 3D-2D estimation using both points and lines
        if (auto pose = _combined_estimator->estimate_3d2d(
            points3d_0,
            tracked_1.keypoints[0],
            tracked_1.lines3d,
            tracked_1.keylines[0]))
        {
            result.pose_3d2d = *pose;
            result.pose_3d2d_lines = *pose;  // Backward compatibility
        }

        // Choose the pose with more inliers
        const auto inliers_3d3d = result.pose_3d3d.inliers.size();
        const auto inliers_3d2d = result.pose_3d2d.inliers.size();

        if (inliers_3d3d >= inliers_3d2d && inliers_3d3d > 0)
        {
            result.chosen_pose = result.pose_3d3d.pose;
            result.chosen_count = inliers_3d3d;
        }
        else if (inliers_3d2d > 0)
        {
            result.chosen_pose = result.pose_3d2d.pose;
            result.chosen_count = inliers_3d2d;
        }
        else
        {
            result.chosen_pose = cv::Affine3d::Identity();
            result.chosen_count = 0;
        }

        return result;
    }

    auto estimator::estimate_pose(
        const frame::estimated& frame_0,
        const frame::tracked& tracked_1) const -> estimate_pose_result
    {
        estimate_pose_result result{};

        // Combined 3D-3D estimation
        if (auto pose = _combined_estimator->estimate_3d3d(
            frame_0.points3d,
            tracked_1.points3d,
            frame_0.lines3d,
            tracked_1.lines3d))
        {
            result.pose_3d3d = *pose;
            result.pose_3d3d_lines = *pose;  // Backward compatibility
        }

        // Combined 3D-2D estimation
        if (auto pose = _combined_estimator->estimate_3d2d(
            frame_0.points3d,
            tracked_1.keypoints[0],
            frame_0.lines3d,
            tracked_1.keylines[0]))
        {
            result.pose_3d2d = *pose;
            result.pose_3d2d_lines = *pose;  // Backward compatibility
        }

        // Combined 2D-2D estimation
        if (auto pose = _combined_estimator->estimate_2d2d(
            frame_0.keypoints[0],
            tracked_1.keypoints[0],
            frame_0.points3d,
            frame_0.lines3d,
            frame_0.keylines[0],
            tracked_1.keylines[0]))
        {
            result.pose_2d2d = *pose;
        }

        // Choose best pose based on inlier count with tie-breaker preference
        const auto in3d3d = result.pose_3d3d.inliers.size();
        const auto in3d2d = result.pose_3d2d.inliers.size();
        const auto in2d2d = result.pose_2d2d.inliers.size();

        size_t best_inliers = 0;
        cv::Affine3d best_pose = cv::Affine3d::Identity();

        // Prefer 3D-3D > 3D-2D > 2D-2D when inlier counts are equal
        if (in3d3d > best_inliers)
        {
            best_inliers = in3d3d;
            best_pose = result.pose_3d3d.pose;
        }

        if (in3d2d > best_inliers || (in3d2d == best_inliers && best_inliers > 0 && in3d2d > 0))
        {
            best_inliers = in3d2d;
            best_pose = result.pose_3d2d.pose;
        }

        if (in2d2d > best_inliers || (in2d2d == best_inliers && best_inliers > 0 && in2d2d > 0))
        {
            best_inliers = in2d2d;
            best_pose = result.pose_2d2d.pose;
        }

        result.chosen_pose = best_pose;
        result.chosen_count = best_inliers;

        return result;
    }

    auto estimator::estimate_pose_weighted(const estimate_pose_result& result) const -> weighted_pose_result
    {
        return pose_estimation::pose_fusion::fuse_poses(result);
    }

} // namespace zenslam
