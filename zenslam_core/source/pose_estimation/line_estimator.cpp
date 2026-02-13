#include "zenslam/pose_estimation/line_estimator.h"

#include <set>
#include <gsl/narrow>
#include <spdlog/spdlog.h>

#include "zenslam/utils_slam.h"

namespace zenslam::pose_estimation
{
    auto line_estimator::estimate_3d2d(
        const std::map<size_t, line3d>& map_lines_0,
        const std::map<size_t, keyline>& map_keylines_1) const -> std::optional<pose_data>
    {
        std::vector<cv::Point3d> lines3d_p1, lines3d_p2;
        std::vector<cv::Point2d> keylines2d_p1, keylines2d_p2;
        std::vector<size_t> indices;

        utils::correspondences_3d2d_lines(
            map_lines_0, map_keylines_1, 
            lines3d_p1, lines3d_p2, 
            keylines2d_p1, keylines2d_p2, 
            indices);

        if (lines3d_p1.size() < 3)
        {
            SPDLOG_DEBUG("Insufficient 3D-2D line correspondences: {} (need >= 3)", lines3d_p1.size());
            return std::nullopt;
        }

        // Combine endpoints into flat point arrays
        std::vector<cv::Point3d> all_points3d;
        std::vector<cv::Point2d> all_points2d;
        all_points3d.reserve(lines3d_p1.size() * 2);
        all_points2d.reserve(keylines2d_p1.size() * 2);

        for (size_t i = 0; i < lines3d_p1.size(); ++i)
        {
            all_points3d.push_back(lines3d_p1[i]);
            all_points3d.push_back(lines3d_p2[i]);
            all_points2d.push_back(keylines2d_p1[i]);
            all_points2d.push_back(keylines2d_p2[i]);
        }

        // Run PnP RANSAC on endpoints using slam_options directly
        const cv::Mat K(_calibration.camera_matrix[0]);
        auto pnp = solve_pnp_ransac(all_points3d, all_points2d, K, _options);

        if (!pnp.success)
        {
            SPDLOG_DEBUG("3D-2D line PnP RANSAC failed");
            return std::nullopt;
        }

        // Map endpoint inliers back to line indices
        std::set<size_t> unique_line_indices;
        for (auto endpoint_idx : pnp.inliers)
        {
            const size_t line_idx = endpoint_idx / 2;
            if (line_idx < indices.size())
                unique_line_indices.insert(indices[line_idx]);
        }

        pose_data out;
        out.pose = cv::Affine3d(pnp.rvec, pnp.tvec);
        out.indices = indices;
        out.inliers.assign(unique_line_indices.begin(), unique_line_indices.end());

        // Outliers
        const std::set<size_t> inlier_set(unique_line_indices.begin(), unique_line_indices.end());
        for (const auto& idx : indices)
        {
            if (!inlier_set.contains(idx))
                out.outliers.push_back(idx);
        }

        // Reprojection errors for endpoints
        out.errors = compute_reprojection_errors(
            all_points3d, all_points2d, pnp.inliers,
            pnp.rvec, pnp.tvec, K);

        SPDLOG_DEBUG("3D-2D lines: {} lines ({} endpoints), {} line inliers", 
            lines3d_p1.size(), all_points3d.size(), unique_line_indices.size());

        return out;
    }

    auto line_estimator::estimate_3d3d(
        const std::map<size_t, line3d>& map_lines_0,
        const std::map<size_t, line3d>& map_lines_1) const -> std::optional<pose_data>
    {
        std::vector<cv::Point3d> lines3d_0_p1, lines3d_0_p2;
        std::vector<cv::Point3d> lines3d_1_p1, lines3d_1_p2;
        std::vector<size_t> indices;

        utils::correspondences_3d3d_lines(
            map_lines_0, map_lines_1,
            lines3d_0_p1, lines3d_0_p2,
            lines3d_1_p1, lines3d_1_p2,
            indices);

        if (lines3d_0_p1.size() < 2)
        {
            SPDLOG_DEBUG("Insufficient 3D-3D line correspondences: {} (need >= 2)", lines3d_0_p1.size());
            return std::nullopt;
        }

        // Combine endpoints into flat point arrays
        std::vector<cv::Point3d> all_points_0, all_points_1;
        all_points_0.reserve(lines3d_0_p1.size() * 2);
        all_points_1.reserve(lines3d_1_p1.size() * 2);

        for (size_t i = 0; i < lines3d_0_p1.size(); ++i)
        {
            all_points_0.push_back(lines3d_0_p1[i]);
            all_points_0.push_back(lines3d_0_p2[i]);
            all_points_1.push_back(lines3d_1_p1[i]);
            all_points_1.push_back(lines3d_1_p2[i]);
        }

        // Estimate rigid transformation
        cv::Matx33d R;
        cv::Point3d t;
        std::vector<size_t> endpoint_inliers, endpoint_outliers;
        std::vector<double> errors;

        utils::estimate_rigid_ransac(
            all_points_0, all_points_1, R, t,
            endpoint_inliers, endpoint_outliers, errors,
            _options.threshold_3d3d, 1000);

        // Map endpoint indices back to line indices
        std::set<size_t> unique_line_inliers, unique_line_outliers;

        for (const auto& endpoint_idx : endpoint_inliers)
        {
            const size_t line_idx = endpoint_idx / 2;
            if (line_idx < indices.size())
                unique_line_inliers.insert(indices[line_idx]);
        }

        for (const auto& endpoint_idx : endpoint_outliers)
        {
            const size_t line_idx = endpoint_idx / 2;
            if (line_idx < indices.size())
                unique_line_outliers.insert(indices[line_idx]);
        }

        SPDLOG_DEBUG("3D-3D lines: {} lines ({} endpoints), {} line inliers",
            lines3d_0_p1.size(), all_points_0.size(), unique_line_inliers.size());

        return pose_data{
            .pose = cv::Affine3d{R, t},
            .indices = indices,
            .inliers = std::vector<size_t>(unique_line_inliers.begin(), unique_line_inliers.end()),
            .outliers = std::vector<size_t>(unique_line_outliers.begin(), unique_line_outliers.end()),
            .errors = errors
        };
    }

} // namespace zenslam::pose_estimation
