#pragma once

#include <map>
#include <optional>
#include <opencv2/core.hpp>

#include "zenslam/calibration.h"
#include "zenslam/pose_data.h"
#include "zenslam/slam_options.h"
#include "zenslam/types/keyline.h"
#include "zenslam/types/line3d.h"
#include "zenslam/pose_estimation/common.h"

namespace zenslam::pose_estimation
{
    /// Specialized estimator for line-based pose estimation
    /// Handles 3D-2D and 3D-3D estimation using line endpoints
    class line_estimator
    {
    public:
        explicit line_estimator(const calibration& calib, const slam_options& opts)
            : _calibration(calib), _options(opts)
        {
        }

        /// Estimate pose using 3D-2D line correspondences
        /// Treats line endpoints as point correspondences for PnP
        [[nodiscard]] auto estimate_3d2d(
            const std::map<size_t, line3d>& map_lines_0,
            const std::map<size_t, keyline>& map_keylines_1) const -> std::optional<pose_data>;

        /// Estimate pose using 3D-3D line correspondences
        /// Treats line endpoints as point correspondences for rigid transformation
        [[nodiscard]] auto estimate_3d3d(
            const std::map<size_t, line3d>& map_lines_0,
            const std::map<size_t, line3d>& map_lines_1) const -> std::optional<pose_data>;

    private:
        const calibration& _calibration;
        const slam_options& _options;
    };

} // namespace zenslam::pose_estimation
