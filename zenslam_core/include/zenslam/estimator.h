#pragma once

#include <map>

#include "zenslam/calibration.h"
#include "zenslam/frame/estimated.h"
#include "zenslam/frame/tracked.h"
#include "zenslam/options.h"
#include "zenslam/pose_data.h"
#include "zenslam/types/keyline.h"
#include "zenslam/types/line3d.h"
#include "zenslam/types/point3d.h"

namespace zenslam
{
    struct estimate_pose_result
    {
        pose_data    pose_3d3d;       // 3D-3D point result (maybe empty)
        pose_data    pose_3d2d;       // 3D-2D point result (maybe empty)
        pose_data    pose_2d2d;       // 2D-2D point result scaled using previous triangulated points (maybe empty)
        pose_data    pose_3d3d_lines; // 3D-3D line result (maybe empty)
        pose_data    pose_3d2d_lines; // 3D-2D line result (maybe empty)
        cv::Affine3d chosen_pose;     // Selected pose (or identity)
        size_t       chosen_count { };
    };

    /// Result from weighted pose fusion combining multiple estimation methods
    struct weighted_pose_result
    {
        cv::Affine3d pose;                    // Fused pose estimate
        double       confidence { 0.0 };      // Overall confidence score (0-1)
        int          total_inliers { 0 };     // Total inliers across all methods
        
        // Individual method contributions
        double       weight_3d3d { 0.0 };
        double       weight_3d2d { 0.0 };
        double       weight_2d2d { 0.0 };
        double       weight_3d3d_lines { 0.0 };
        double       weight_3d2d_lines { 0.0 };
        
        // Best contributing method
        std::string  best_method;
        size_t       best_method_inliers { 0 };
        
        // Uncertainty quantification
        cv::Matx66d  pose_covariance { };              // 6x6 uncertainty matrix (SE(3))
        double       translation_std { 0.0 };          // Translation std dev (meters)
        double       rotation_std { 0.0 };             // Rotation std dev (radians)
        bool         has_valid_covariance { false };   // Whether covariance is valid
    };

    class estimator
    {
    public:
        estimator(calibration calib, slam_options opts);

        /** Estimate the pose of frame 1 relative to frame 0 using 3D-2D and 3D-3D correspondences.
         *
         * @param points3d_0 Map of 3D points in frame 0
         * @param tracked_1 Tracked frame 1 containing keypoints and/or 3D points
         * @return Struct containing both pose estimates and the chosen pose
         */
        [[nodiscard]] auto estimate_pose(const std::map<size_t, point3d>& points3d_0, const frame::tracked& tracked_1) const -> estimate_pose_result;

        /** Estimate pose using 2D-2D essential matrix and scale from previous triangulated points, alongside 3D-2D and 3D-3D.
         *
         * @param frame_0 Previous estimated frame (provides previous keypoints and 3D points)
         * @param tracked_1 Current tracked frame
         */
        [[nodiscard]] auto estimate_pose(const frame::estimated& frame_0, const frame::tracked& tracked_1) const -> estimate_pose_result;

        /** Compute weighted fusion of multiple pose estimation methods.
         *
         * Combines all available pose estimates (3D-3D, 3D-2D, 2D-2D, lines) using confidence weights based on:
         * - Inlier ratio (inliers vs total correspondences)
         * - Mean reprojection error (lower error = higher confidence)
         * - Feature type reliability (points > lines)
         *
         * @param result Standard pose estimation result containing all methods
         * @return Fused pose with confidence scores and method contributions
         */
        [[nodiscard]] auto estimate_pose_weighted(const estimate_pose_result& result) const -> weighted_pose_result;

    private:
        [[nodiscard]] auto estimate_pose_3d2d(const std::map<size_t, point3d>& map_points_0, const std::map<size_t, keypoint>& map_keypoints_1) const
            -> pose_data;

        [[nodiscard]] auto estimate_pose_3d3d(const std::map<size_t, point3d>& map_points_0, const std::map<size_t, point3d>& map_points_1) const -> pose_data;

        [[nodiscard]] auto estimate_pose_2d2d(
            const std::map<size_t, keypoint>& map_keypoints_0,
            const std::map<size_t, keypoint>& map_keypoints_1,
            const std::map<size_t, point3d>&  map_points3d_0) const -> pose_data;

        [[nodiscard]] auto estimate_pose_3d2d_lines(const std::map<size_t, line3d>& map_lines_0, const std::map<size_t, keyline>& map_keylines_1) const
            -> pose_data;

        [[nodiscard]] auto estimate_pose_3d3d_lines(const std::map<size_t, line3d>& map_lines_0, const std::map<size_t, line3d>& map_lines_1) const -> pose_data;

        /** Combined 3D-2D pose estimation using both points and line endpoints.
         * 
         * Treats line endpoints as additional point correspondences, increasing the constraint count
         * for more robust RANSAC estimation. This is more efficient and robust than separate estimation.
         * 
         * @param map_points_0 3D points from previous frame
         * @param map_keypoints_1 2D keypoints in current frame
         * @param map_lines_0 3D lines from previous frame
         * @param map_keylines_1 2D keylines in current frame
         * @return Unified pose estimate with all inliers
         */
        [[nodiscard]] auto estimate_pose_3d2d_combined(
            const std::map<size_t, point3d>& map_points_0,
            const std::map<size_t, keypoint>& map_keypoints_1,
            const std::map<size_t, line3d>& map_lines_0,
            const std::map<size_t, keyline>& map_keylines_1) const -> pose_data;

        /** Combined 3D-3D pose estimation using both points and line endpoints.
         * 
         * @param map_points_0 3D points from frame 0
         * @param map_points_1 3D points from frame 1
         * @param map_lines_0 3D lines from frame 0
         * @param map_lines_1 3D lines from frame 1
         * @return Unified pose estimate with all inliers
         */
        [[nodiscard]] auto estimate_pose_3d3d_combined(
            const std::map<size_t, point3d>& map_points_0,
            const std::map<size_t, point3d>& map_points_1,
            const std::map<size_t, line3d>& map_lines_0,
            const std::map<size_t, line3d>& map_lines_1) const -> pose_data;

        /** Combined 2D-2D pose estimation using both points and line endpoints.
         * 
         * @param map_keypoints_0 2D keypoints from frame 0
         * @param map_keypoints_1 2D keypoints from frame 1
         * @param map_points3d_0 3D points from frame 0 for scale recovery
         * @param map_lines_0 3D lines from frame 0 for line endpoint scale recovery
         * @param map_keylines_0 2D keylines from frame 0
         * @param map_keylines_1 2D keylines from frame 1
         * @return Unified pose estimate with all inliers
         */
        [[nodiscard]] auto estimate_pose_2d2d_combined(
            const std::map<size_t, keypoint>& map_keypoints_0,
            const std::map<size_t, keypoint>& map_keypoints_1,
            const std::map<size_t, point3d>& map_points3d_0,
            const std::map<size_t, line3d>& map_lines_0,
            const std::map<size_t, keyline>& map_keylines_0,
            const std::map<size_t, keyline>& map_keylines_1) const -> pose_data;

    private:
        calibration  _calibration;
        slam_options _options;
    };
} // namespace zenslam
