#pragma once

#include <map>
#include <memory>

#include "zenslam/calibration/calibration.h"
#include "zenslam/frame/estimated.h"
#include "zenslam/frame/tracked.h"
#include "zenslam/options.h"
#include "zenslam/pose_estimation/pose_fusion.h"

// Forward declarations to avoid including heavy headers
namespace zenslam::pose_estimation
{
    class point_estimator;
    class line_estimator;
    class combined_estimator;
}

namespace zenslam
{
    // Re-export for backward compatibility
    using estimate_pose_result = pose_estimation::estimate_pose_result;
    using weighted_pose_result = pose_estimation::weighted_pose_result;

    /// High-level pose estimator using modular estimation strategies
    /// Delegates to specialized estimators for points, lines, and combined features
    class estimator
    {
    public:
        estimator(calibration calib, slam_options opts);
        ~estimator();

        // Prevent copies, allow moves
        estimator(const estimator&)            = delete;
        estimator& operator=(const estimator&) = delete;
        estimator(estimator&&) noexcept;
        estimator& operator=(estimator&&) noexcept;

        /// Estimate pose using 3D points and tracked features
        /// @param points3d_0 Map of 3D points in frame 0
        /// @param tracked_1 Tracked frame 1 containing keypoints and/or 3D points
        /// @return Struct containing all pose estimates and the chosen pose
        [[nodiscard]] auto estimate_pose
        (
            const std::map<size_t, point3d>& points3d_0,
            const frame::tracked&            tracked_1
        ) const
            -> estimate_pose_result;

        /// Estimate pose using full frame information (points and lines)
        /// @param frame_0 Previous estimated frame
        /// @param tracked_1 Current tracked frame
        /// @return Struct containing all pose estimates and the chosen pose
        [[nodiscard]] auto estimate_pose
        (
            const frame::estimated& frame_0,
            const frame::tracked&   tracked_1
        ) const
            -> estimate_pose_result;
        pose_data estimate_pose_3d2d(const frame::estimated& frame_0, const frame::tracked& tracked_1, const size_t& camera_index) const;

        [[nodiscard]] auto estimate_pose_new
        (
            const frame::estimated& frame_0,
            const frame::tracked&   tracked_1
        ) const
            -> estimate_pose_result;

        /// Compute weighted fusion of multiple pose estimation methods
        /// @param result Standard pose estimation result containing all methods
        /// @return Fused pose with confidence scores and method contributions
        [[nodiscard]] static auto estimate_pose_weighted
        (
            const estimate_pose_result& result
        )
            -> weighted_pose_result;

    private:
        calibration  _calibration;
        slam_options _options;

        // Strategy pattern: delegate to specialized estimators
        std::unique_ptr<pose_estimation::point_estimator>    _point_estimator;
        std::unique_ptr<pose_estimation::line_estimator>     _line_estimator;
        std::unique_ptr<pose_estimation::combined_estimator> _combined_estimator;

        auto solvepnp_ransac(const std::vector<std::pair<point3d, keypoint>>& correspondences) const
            -> pose_data;
    };
} // namespace zenslam
