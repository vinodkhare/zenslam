#pragma once

#include <map>
#include <optional>

#include "zenslam/calibration.h"
#include "zenslam/pose_data.h"
#include "zenslam/slam_options.h"
#include "zenslam/types/keypoint.h"
#include "zenslam/types/point3d.h"

namespace zenslam::pose_estimation
{
    /// Specialized estimator for point-based pose estimation
    /// Handles 3D-2D, 3D-3D, and 2D-2D estimation methods
    class point_estimator
    {
    public:
        explicit point_estimator(const calibration& calib, const slam_options& opts)
            : _calibration(calib), _options(opts)
        {
        }

        /// Estimate pose using 3D-2D point correspondences (PnP)
        [[nodiscard]] auto estimate_3d2d(
            const std::map<size_t, point3d>& map_points_0,
            const std::map<size_t, keypoint>& map_keypoints_1) const -> std::optional<pose_data>;

        /// Estimate pose using 3D-3D point correspondences
        [[nodiscard]] auto estimate_3d3d(
            const std::map<size_t, point3d>& map_points_0,
            const std::map<size_t, point3d>& map_points_1) const -> std::optional<pose_data>;

        /// Estimate pose using 2D-2D correspondences with scale recovery
        /// @param map_points3d_0 3D points from previous frame for scale estimation
        [[nodiscard]] auto estimate_2d2d(
            const std::map<size_t, keypoint>& map_keypoints_0,
            const std::map<size_t, keypoint>& map_keypoints_1,
            const std::map<size_t, point3d>& map_points3d_0) const -> std::optional<pose_data>;

    private:
        const calibration& _calibration;
        const slam_options& _options;
    };

} // namespace zenslam::pose_estimation
