#pragma once

#include <map>
#include <optional>
#include <set>

#include "zenslam/calibration/calibration.h"
#include "zenslam/utils/pose_data.h"
#include "zenslam/all_options.h"
#include "zenslam/types/keypoint.h"
#include "zenslam/types/keyline.h"
#include "zenslam/types/point3d.h"
#include "zenslam/types/line3d.h"

namespace zenslam::pose_estimation
{
    /// Combined estimator that fuses points and lines for robust pose estimation
    /// Treats line endpoints as additional point correspondences
    class combined_estimator
    {
    public:
        explicit combined_estimator(const calibration& calib, const slam_options& opts)
            : _calibration(calib), _options(opts)
        {
        }

        /// Combined 3D-2D estimation using both points and line endpoints
        [[nodiscard]] auto estimate_3d2d(
            const std::map<size_t, point3d>& map_points_0,
            const std::map<size_t, keypoint>& map_keypoints_1,
            const std::map<size_t, line3d>& map_lines_0,
            const std::map<size_t, keyline>& map_keylines_1) const -> std::optional<pose_data>;

        /// Combined 3D-3D estimation using both points and line endpoints
        [[nodiscard]] auto estimate_3d3d(
            const std::map<size_t, point3d>& map_points_0,
            const std::map<size_t, point3d>& map_points_1,
            const std::map<size_t, line3d>& map_lines_0,
            const std::map<size_t, line3d>& map_lines_1) const -> std::optional<pose_data>;

        /// Combined 2D-2D estimation using both points and line endpoints
        [[nodiscard]] auto estimate_2d2d(
            const std::map<size_t, keypoint>& map_keypoints_0,
            const std::map<size_t, keypoint>& map_keypoints_1,
            const std::map<size_t, point3d>& map_points3d_0,
            const std::map<size_t, line3d>& map_lines_0,
            const std::map<size_t, keyline>& map_keylines_0,
            const std::map<size_t, keyline>& map_keylines_1) const -> std::optional<pose_data>;

    private:
        /// Helper: Separate mixed feature inliers back to point and line IDs
        struct feature_split
        {
            std::set<size_t> point_ids;
            std::set<size_t> line_ids;
        };

        [[nodiscard]] static auto split_feature_inliers(
            const std::vector<int>& correspondence_inliers,
            size_t num_points,
            const std::vector<size_t>& point_indices,
            const std::vector<size_t>& line_indices) -> feature_split;

        const calibration& _calibration;
        const slam_options& _options;
    };

} // namespace zenslam::pose_estimation
