#include "zenslam/pose_estimation/point_estimator.h"

#include <algorithm>
#include <set>

#include <gsl/narrow>

#include <spdlog/spdlog.h>

#include "zenslam/rigid_transform.h"
#include "zenslam/triangulation_utils.h"
#include "zenslam/utils_slam.h"
#include "zenslam/utils_std.h"
#include "zenslam/pose_estimation/common.h"

namespace zenslam::pose_estimation
{
    auto point_estimator::estimate_3d2d
    (
        const std::map<size_t, point3d>&  map_points_0,
        const std::map<size_t, keypoint>& map_keypoints_1
    )
    const -> std::optional<pose_data>
    {
        std::vector<cv::Point3d> points3d;
        std::vector<cv::Point2d> points2d;
        std::vector<size_t>      indices;

        utils::correspondences_3d2d(map_points_0, map_keypoints_1, points3d, points2d, indices);

        if (points3d.size() < 6)
        {
            SPDLOG_DEBUG("Insufficient 3D-2D point correspondences: {} (need >= 6)", points3d.size());
            return std::nullopt;
        }

        // Run PnP RANSAC using slam_options directly
        const cv::Mat K(_calibration.camera_matrix[0]);
        auto          [rvec, tvec, inliers, success] = solve_pnp_ransac(points3d, points2d, K, _options);

        if (!success)
        {
            SPDLOG_DEBUG("3D-2D PnP RANSAC failed");
            return std::nullopt;
        }

        // Build pose_data result using modern C++23 features
        pose_data out;
        out.pose    = cv::Affine3d(rvec, tvec);
        out.indices = indices;
        out.inliers = map_to_feature_space(inliers, indices);

        // Compute outliers using set difference
        const auto inliers_set = std::set(out.inliers.begin(), out.inliers.end());
        for (auto idx : indices)
        {
            if (!inliers_set.contains(idx))
                out.outliers.push_back(idx);
        }

        // Compute reprojection errors
        out.errors = compute_reprojection_errors(
            points3d, points2d, inliers,
            rvec, tvec, K);

        SPDLOG_DEBUG("3D-2D points: {} correspondences, {} inliers", points3d.size(), out.inliers.size());

        return out;
    }

    auto point_estimator::estimate_3d3d
    (
        const std::map<size_t, point3d>& map_points_0,
        const std::map<size_t, point3d>& map_points_1
    )
    const -> std::optional<pose_data>
    {
        std::vector<cv::Point3d> points_0;
        std::vector<cv::Point3d> points_1;
        std::vector<size_t>      indices;
        utils::correspondences_3d3d(map_points_0, map_points_1, points_0, points_1, indices);

        if (points_0.size() < 3)
        {
            SPDLOG_DEBUG("Insufficient 3D-3D point correspondences: {} (need >= 3)", points_0.size());
            return std::nullopt;
        }

        cv::Matx33d         R;
        cv::Point3d         t;
        std::vector<size_t> inliers, outliers;
        std::vector<double> errors;

        utils::estimate_rigid_ransac(
            points_0, points_1, R, t,
            inliers, outliers, errors,
            _options.threshold_3d3d, 1000);

        SPDLOG_DEBUG("3D-3D points: {} correspondences, {} inliers",
                     points_0.size(), inliers.size());

        return pose_data {
            .pose     = cv::Affine3d { R, t },
            .indices  = indices,
            .inliers  = inliers,
            .outliers = outliers,
            .errors   = errors
        };
    }

    auto point_estimator::estimate_2d2d(
        const std::map<size_t, keypoint>& map_keypoints_0,
        const std::map<size_t, keypoint>& map_keypoints_1,
        const std::map<size_t, point3d>&  map_points3d_0) const -> std::optional<pose_data>
    {
        // Build 2D-2D correspondences
        std::vector<cv::Point2f> points0, points1;
        std::vector<size_t>      indices;
        utils::correspondence_2d2d(map_keypoints_0, map_keypoints_1, points0, points1, indices);

        if (points0.size() < 8)
        {
            SPDLOG_DEBUG("Insufficient 2D-2D point correspondences: {} (need >= 8)", points0.size());
            return std::nullopt;
        }

        // Estimate essential matrix
        std::vector<uchar> inlier_mask;
        cv::Mat            E = cv::findEssentialMat(
            points0, points1,
            _calibration.camera_matrix[0],
            cv::RANSAC, 0.999,
            _options.epipolar_threshold,
            inlier_mask);

        if (E.empty())
        {
            SPDLOG_DEBUG("Essential matrix estimation failed");
            return std::nullopt;
        }

        // Recover pose
        cv::Mat R, t;
        int     inliers = cv::recoverPose(
            E, points0, points1,
            _calibration.camera_matrix[0],
            R, t, inlier_mask);

        if (inliers < 5)
        {
            SPDLOG_DEBUG("Pose recovery produced insufficient inliers: {}", inliers);
            return std::nullopt;
        }

        // Gather inlier correspondences with 3D points for scale estimation
        std::vector<cv::Point2f> inlier_p0, inlier_p1;
        std::vector<cv::Point3d> X0_list;

        for (size_t i = 0; i < indices.size(); ++i)
        {
            if (!inlier_mask[i])
                continue;

            const auto idx = indices[i];
            if (auto itX = map_points3d_0.find(idx); itX != map_points3d_0.end())
            {
                inlier_p0.emplace_back(points0[i]);
                inlier_p1.emplace_back(points1[i]);
                X0_list.emplace_back(itX->second);
            }
        }

        if (X0_list.size() < 5)
        {
            SPDLOG_DEBUG("Insufficient inliers with 3D points for scale: {}", X0_list.size());
            return std::nullopt;
        }

        // Triangulate and compute scale
        const cv::Matx33d K(_calibration.camera_matrix[0]);
        const auto        Rm = cv::Matx33d(R);
        const auto        tv = cv::Vec3d(t);

        cv::Matx34d P0(
            K(0, 0), K(0, 1), K(0, 2), 0.0,
            K(1, 0), K(1, 1), K(1, 2), 0.0,
            K(2, 0), K(2, 1), K(2, 2), 0.0);

        const cv::Matx33d KR = K * Rm;
        const cv::Vec3d   Kt = K * tv;
        cv::Matx34d       P1(
            KR(0, 0), KR(0, 1), KR(0, 2), Kt[0],
            KR(1, 0), KR(1, 1), KR(1, 2), Kt[1],
            KR(2, 0), KR(2, 1), KR(2, 2), Kt[2]);

        const auto X_est = utils::triangulate_points(inlier_p0, inlier_p1, P0, P1);

        // Compute robust scale as median of ||X0|| / ||X_est||
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
            SPDLOG_DEBUG("Insufficient valid scales computed: {}", scales.size());
            return std::nullopt;
        }

        const double      s        = utils::median(scales);
        const cv::Point3d t_scaled = cv::Point3d(tv) * s;

        // Build result
        pose_data out;
        out.pose    = cv::Affine3d(Rm, t_scaled);
        out.indices = indices;

        // Split inliers/outliers
        for (size_t i = 0; i < indices.size(); ++i)
        {
            if (inlier_mask[i])
                out.inliers.push_back(indices[i]);
            else
                out.outliers.push_back(indices[i]);
        }

        // Reprojection errors
        std::vector<cv::Point2d> proj;
        cv::Mat                  rvec;
        cv::Rodrigues(R, rvec);
        cv::projectPoints(X0_list, rvec, cv::Mat(t_scaled),
                          _calibration.camera_matrix[0], cv::Mat(), proj);

        for (size_t i = 0; i < proj.size(); ++i)
        {
            out.errors.push_back(cv::norm(proj[i] - cv::Point2d(inlier_p1[i])));
        }

        SPDLOG_DEBUG("2D-2D points: {} correspondences, {} inliers, scale={:.3f}",
                     points0.size(), out.inliers.size(), s);

        return out;
    }
} // namespace zenslam::pose_estimation
