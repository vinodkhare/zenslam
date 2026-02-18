#include "zenslam/pose_estimation/combined_estimator.h"

#include <algorithm>
#include <set>
#include <gsl/narrow>
#include <spdlog/spdlog.h>

#include "zenslam/utils_slam.h"
#include "zenslam/utils_std.h"
#include "zenslam/pose_estimation/common.h"

namespace zenslam::pose_estimation
{
    auto combined_estimator::split_feature_inliers(
        const std::vector<int>& correspondence_inliers,
        const size_t num_points,
        const std::vector<size_t>& point_indices,
        const std::vector<size_t>& line_indices) -> feature_split
    {
        feature_split result;

        for (auto idx : correspondence_inliers)
        {
            if (idx < static_cast<int>(num_points))
            {
                // Point inlier
                result.point_ids.insert(point_indices[idx]);
            }
            else
            {
                // Line endpoint inlier - map back to line ID
                const size_t line_endpoint_idx = idx - num_points;
                const size_t line_idx = line_endpoint_idx / 2;
                if (line_idx < line_indices.size())
                    result.line_ids.insert(line_indices[line_idx]);
            }
        }

        return result;
    }

    auto combined_estimator::estimate_3d2d(
        const std::map<size_t, point3d>& map_points_0,
        const std::map<size_t, keypoint>& map_keypoints_1,
        const std::map<size_t, line3d>& map_lines_0,
        const std::map<size_t, keyline>& map_keylines_1) const -> std::optional<pose_data>
    {
        // Gather point correspondences
        std::vector<cv::Point3d> points3d;
        std::vector<cv::Point2d> points2d;
        std::vector<size_t> point_indices;
        utils::correspondences_3d2d(map_points_0, map_keypoints_1, points3d, points2d, point_indices);

        // Gather line endpoint correspondences
        std::vector<cv::Point3d> lines3d_p1, lines3d_p2;
        std::vector<cv::Point2d> keylines2d_p1, keylines2d_p2;
        std::vector<size_t> line_indices;
        utils::correspondences_3d2d_lines(
            map_lines_0, map_keylines_1,
            lines3d_p1, lines3d_p2,
            keylines2d_p1, keylines2d_p2,
            line_indices);

        // Combine into unified arrays
        std::vector<cv::Point3d> all_points3d;
        std::vector<cv::Point2d> all_points2d;
        all_points3d.reserve(points3d.size() + lines3d_p1.size() * 2);
        all_points2d.reserve(points2d.size() + keylines2d_p1.size() * 2);

        const size_t num_points = points3d.size();
        all_points3d.insert(all_points3d.end(), points3d.begin(), points3d.end());
        all_points2d.insert(all_points2d.end(), points2d.begin(), points2d.end());

        // Add line endpoints
        for (size_t i = 0; i < lines3d_p1.size(); ++i)
        {
            all_points3d.push_back(lines3d_p1[i]);
            all_points3d.push_back(lines3d_p2[i]);
            all_points2d.push_back(keylines2d_p1[i]);
            all_points2d.push_back(keylines2d_p2[i]);
        }

        if (all_points3d.size() < 6)
        {
            SPDLOG_DEBUG("Insufficient combined 3D-2D correspondences: {} (need >= 6)", all_points3d.size());
            return std::nullopt;
        }

        // Run PnP RANSAC using slam_options directly
        const cv::Mat K(_calibration.camera_matrix[0]);
        auto pnp = solve_pnp_ransac(all_points3d, all_points2d, K, _options);

        if (!pnp.success)
        {
            SPDLOG_DEBUG("Combined 3D-2D PnP RANSAC failed");
            return std::nullopt;
        }

        // Split inliers back to feature IDs
        const auto [inlier_point_ids, inlier_line_ids] = 
            split_feature_inliers(pnp.inliers, num_points, point_indices, line_indices);

        pose_data out;
        out.pose = cv::Affine3d(pnp.rvec, pnp.tvec);

        // Combine all feature IDs
        out.indices.reserve(point_indices.size() + line_indices.size());
        out.indices.insert(out.indices.end(), point_indices.begin(), point_indices.end());
        out.indices.insert(out.indices.end(), line_indices.begin(), line_indices.end());

        out.inliers.reserve(inlier_point_ids.size() + inlier_line_ids.size());
        out.inliers.insert(out.inliers.end(), inlier_point_ids.begin(), inlier_point_ids.end());
        out.inliers.insert(out.inliers.end(), inlier_line_ids.begin(), inlier_line_ids.end());

        // Outliers
        for (auto id : point_indices)
            if (!inlier_point_ids.contains(id))
                out.outliers.push_back(id);
        for (auto id : line_indices)
            if (!inlier_line_ids.contains(id))
                out.outliers.push_back(id);

        // Compute errors
        out.errors = compute_reprojection_errors(
            all_points3d, all_points2d, pnp.inliers,
            pnp.rvec, pnp.tvec, K);

        SPDLOG_DEBUG("Combined 3D-2D: {} total ({} points + {} lines), {} inliers ({} point + {} line features)",
            all_points3d.size(), num_points, lines3d_p1.size(),
            pnp.inliers.size(), inlier_point_ids.size(), inlier_line_ids.size());

        return out;
    }

    auto combined_estimator::estimate_3d3d(
        const std::map<size_t, point3d>& map_points_0,
        const std::map<size_t, point3d>& map_points_1,
        const std::map<size_t, line3d>& map_lines_0,
        const std::map<size_t, line3d>& map_lines_1) const -> std::optional<pose_data>
    {
        // Gather point correspondences
        std::vector<cv::Point3d> points_0, points_1;
        std::vector<size_t> point_indices;
        utils::correspondences_3d3d(map_points_0, map_points_1, points_0, points_1, point_indices);

        // Gather line endpoint correspondences
        std::vector<cv::Point3d> lines3d_0_p1, lines3d_0_p2;
        std::vector<cv::Point3d> lines3d_1_p1, lines3d_1_p2;
        std::vector<size_t> line_indices;
        utils::correspondences_3d3d_lines(
            map_lines_0, map_lines_1,
            lines3d_0_p1, lines3d_0_p2,
            lines3d_1_p1, lines3d_1_p2,
            line_indices);

        // Combine into unified arrays
        std::vector<cv::Point3d> all_points_0, all_points_1;
        all_points_0.reserve(points_0.size() + lines3d_0_p1.size() * 2);
        all_points_1.reserve(points_1.size() + lines3d_1_p1.size() * 2);

        const size_t num_points = points_0.size();
        all_points_0.insert(all_points_0.end(), points_0.begin(), points_0.end());
        all_points_1.insert(all_points_1.end(), points_1.begin(), points_1.end());

        // Add line endpoints
        for (size_t i = 0; i < lines3d_0_p1.size(); ++i)
        {
            all_points_0.push_back(lines3d_0_p1[i]);
            all_points_0.push_back(lines3d_0_p2[i]);
            all_points_1.push_back(lines3d_1_p1[i]);
            all_points_1.push_back(lines3d_1_p2[i]);
        }

        if (all_points_0.size() < 3)
        {
            SPDLOG_DEBUG("Insufficient combined 3D-3D correspondences: {} (need >= 3)", all_points_0.size());
            return std::nullopt;
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

        // Convert inliers from int to size_t for split_feature_inliers
        std::vector<int> inliers_int(endpoint_inliers.begin(), endpoint_inliers.end());
        const auto [inlier_point_ids, inlier_line_ids] = 
            split_feature_inliers(inliers_int, num_points, point_indices, line_indices);

        // Do the same for outliers
        std::vector<int> outliers_int(endpoint_outliers.begin(), endpoint_outliers.end());
        const auto [outlier_point_ids, outlier_line_ids] = 
            split_feature_inliers(outliers_int, num_points, point_indices, line_indices);

        pose_data out;
        out.pose = cv::Affine3d{R, t};

        // Combine all feature IDs
        out.indices.reserve(point_indices.size() + line_indices.size());
        out.indices.insert(out.indices.end(), point_indices.begin(), point_indices.end());
        out.indices.insert(out.indices.end(), line_indices.begin(), line_indices.end());

        out.inliers.reserve(inlier_point_ids.size() + inlier_line_ids.size());
        out.inliers.insert(out.inliers.end(), inlier_point_ids.begin(), inlier_point_ids.end());
        out.inliers.insert(out.inliers.end(), inlier_line_ids.begin(), inlier_line_ids.end());

        out.outliers.reserve(outlier_point_ids.size() + outlier_line_ids.size());
        out.outliers.insert(out.outliers.end(), outlier_point_ids.begin(), outlier_point_ids.end());
        out.outliers.insert(out.outliers.end(), outlier_line_ids.begin(), outlier_line_ids.end());

        out.errors = errors;

        SPDLOG_DEBUG("Combined 3D-3D: {} total ({} points + {} lines), {} inliers ({} point + {} line features)",
            all_points_0.size(), num_points, lines3d_0_p1.size(),
            endpoint_inliers.size(), inlier_point_ids.size(), inlier_line_ids.size());

        return out;
    }

    auto combined_estimator::estimate_2d2d(
        const std::map<size_t, keypoint>& map_keypoints_0,
        const std::map<size_t, keypoint>& map_keypoints_1,
        const std::map<size_t, point3d>& map_points3d_0,
        const std::map<size_t, line3d>& map_lines_0,
        const std::map<size_t, keyline>& map_keylines_0,
        const std::map<size_t, keyline>& map_keylines_1) const -> std::optional<pose_data>
    {
        // Gather point correspondences
        std::vector<cv::Point2f> points0, points1;
        std::vector<size_t> point_indices;
        utils::correspondence_2d2d(map_keypoints_0, map_keypoints_1, points0, points1, point_indices);

        // Gather line endpoint correspondences
        std::vector<cv::Point2f> keylines0_p1, keylines0_p2;
        std::vector<cv::Point2f> keylines1_p1, keylines1_p2;
        std::vector<size_t> line_indices;

        for (const auto& [id0, kl0] : map_keylines_0)
        {
            if (auto it1 = map_keylines_1.find(id0); it1 != map_keylines_1.end())
            {
                const auto& kl1 = it1->second;
                keylines0_p1.emplace_back(kl0.startPointX, kl0.startPointY);
                keylines0_p2.emplace_back(kl0.endPointX, kl0.endPointY);
                keylines1_p1.emplace_back(kl1.startPointX, kl1.startPointY);
                keylines1_p2.emplace_back(kl1.endPointX, kl1.endPointY);
                line_indices.push_back(id0);
            }
        }

        // Combine into unified arrays
        std::vector<cv::Point2f> all_points0, all_points1;
        all_points0.reserve(points0.size() + keylines0_p1.size() * 2);
        all_points1.reserve(points1.size() + keylines1_p1.size() * 2);

        const size_t num_points = points0.size();
        all_points0.insert(all_points0.end(), points0.begin(), points0.end());
        all_points1.insert(all_points1.end(), points1.begin(), points1.end());

        // Add line endpoints
        for (size_t i = 0; i < keylines0_p1.size(); ++i)
        {
            all_points0.push_back(keylines0_p1[i]);
            all_points0.push_back(keylines0_p2[i]);
            all_points1.push_back(keylines1_p1[i]);
            all_points1.push_back(keylines1_p2[i]);
        }

        if (all_points0.size() < 8)
        {
            SPDLOG_DEBUG("Insufficient combined 2D-2D correspondences: {} (need >= 8)", all_points0.size());
            return std::nullopt;
        }

        // Estimate essential matrix
        std::vector<uchar> inlier_mask;
        cv::Mat E = cv::findEssentialMat(
            all_points0, all_points1,
            _calibration.camera_matrix[0],
            cv::RANSAC, 0.999,
            _options.epipolar_threshold,
            inlier_mask);

        if (E.empty())
        {
            SPDLOG_DEBUG("Combined 2D-2D essential matrix estimation failed");
            return std::nullopt;
        }

        // Recover pose
        cv::Mat R, t;
        int inliers = cv::recoverPose(
            E, all_points0, all_points1,
            _calibration.camera_matrix[0],
            R, t, inlier_mask);

        if (inliers < 5)
        {
            SPDLOG_DEBUG("Combined 2D-2D pose recovery insufficient inliers: {}", inliers);
            return std::nullopt;
        }

        // Gather inliers with 3D data for scale estimation
        std::vector<cv::Point2f> inlier_p0, inlier_p1;
        std::vector<cv::Point3d> X0_list;

        for (size_t i = 0; i < all_points0.size(); ++i)
        {
            if (!inlier_mask[i])
                continue;

            cv::Point3d X0;
            bool has_3d = false;

            if (i < num_points)
            {
                // Point inlier
                if (auto itX = map_points3d_0.find(point_indices[i]); itX != map_points3d_0.end())
                {
                    X0 = itX->second;
                    has_3d = true;
                }
            }
            else
            {
                // Line endpoint inlier
                const size_t line_endpoint_idx = i - num_points;
                const size_t line_idx = line_endpoint_idx / 2;
                const bool is_endpoint1 = (line_endpoint_idx % 2) == 0;

                if (line_idx < line_indices.size())
                {
                    if (auto itL = map_lines_0.find(line_indices[line_idx]); itL != map_lines_0.end())
                    {
                        X0 = is_endpoint1 ? itL->second[0] : itL->second[1];
                        has_3d = true;
                    }
                }
            }

            if (has_3d)
            {
                inlier_p0.push_back(all_points0[i]);
                inlier_p1.push_back(all_points1[i]);
                X0_list.push_back(X0);
            }
        }

        if (X0_list.size() < 5)
        {
            SPDLOG_DEBUG("Insufficient inliers with 3D data for scale: {}", X0_list.size());
            return std::nullopt;
        }

        // Triangulate and compute scale
        const cv::Matx33d K(_calibration.camera_matrix[0]);
        const auto Rm = cv::Matx33d(R);
        const auto tv = cv::Vec3d(t);

        cv::Matx34d P0(
            K(0, 0), K(0, 1), K(0, 2), 0.0,
            K(1, 0), K(1, 1), K(1, 2), 0.0,
            K(2, 0), K(2, 1), K(2, 2), 0.0);

        const cv::Matx33d KR = K * Rm;
        const cv::Vec3d Kt = K * tv;
        cv::Matx34d P1(
            KR(0, 0), KR(0, 1), KR(0, 2), Kt[0],
            KR(1, 0), KR(1, 1), KR(1, 2), Kt[1],
            KR(2, 0), KR(2, 1), KR(2, 2), Kt[2]);

        const auto X_est = utils::triangulate_points(inlier_p0, inlier_p1, P0, P1);

        // Compute robust scale
        std::vector<double> scales;
        scales.reserve(X_est.size());

        for (size_t i = 0; i < X_est.size(); ++i)
        {
            const double n_est = cv::norm(X_est[i]);
            const double n_ref = cv::norm(X0_list[i]);
            if (n_est > 1e-9)
            {
                const double s = n_ref / n_est;
                if (std::isfinite(s) && s > 0.0)
                    scales.push_back(s);
            }
        }

        if (scales.size() < 3)
        {
            SPDLOG_DEBUG("Insufficient valid scales: {}", scales.size());
            return std::nullopt;
        }

        const double s = utils::median(scales);
        const cv::Point3d t_scaled = cv::Point3d(tv) * s;

        // Build output - split inliers/outliers by feature type
        std::vector<int> inlier_indices_int;
        for (size_t i = 0; i < all_points0.size(); ++i)
        {
            if (inlier_mask[i])
                inlier_indices_int.push_back(static_cast<int>(i));
        }

        const auto [inlier_point_ids, inlier_line_ids] =
            split_feature_inliers(inlier_indices_int, num_points, point_indices, line_indices);

        pose_data out;
        out.pose = cv::Affine3d(Rm, t_scaled);

        // Combine all feature IDs
        out.indices.reserve(point_indices.size() + line_indices.size());
        out.indices.insert(out.indices.end(), point_indices.begin(), point_indices.end());
        out.indices.insert(out.indices.end(), line_indices.begin(), line_indices.end());

        out.inliers.reserve(inlier_point_ids.size() + inlier_line_ids.size());
        out.inliers.insert(out.inliers.end(), inlier_point_ids.begin(), inlier_point_ids.end());
        out.inliers.insert(out.inliers.end(), inlier_line_ids.begin(), inlier_line_ids.end());

        // Outliers
        for (auto id : point_indices)
            if (!inlier_point_ids.contains(id))
                out.outliers.push_back(id);
        for (auto id : line_indices)
            if (!inlier_line_ids.contains(id))
                out.outliers.push_back(id);

        // Errors
        std::vector<cv::Point2d> proj;
        cv::Mat rvec;
        cv::Rodrigues(R, rvec);
        cv::projectPoints(X0_list, rvec, cv::Mat(t_scaled),
            _calibration.camera_matrix[0], cv::Mat(), proj);

        for (size_t i = 0; i < proj.size(); ++i)
        {
            out.errors.push_back(cv::norm(proj[i] - cv::Point2d(inlier_p1[i])));
        }

        SPDLOG_DEBUG("Combined 2D-2D: {} total ({} points + {} lines), {} inliers ({} point + {} line features), scale={:.3f}",
            all_points0.size(), num_points, keylines0_p1.size(),
            inliers, inlier_point_ids.size(), inlier_line_ids.size(), s);

        return out;
    }

} // namespace zenslam::pose_estimation
