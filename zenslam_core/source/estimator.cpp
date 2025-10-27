#include "zenslam/estimator.h"

#include <set>
#include <algorithm>
#include <cmath>

#include <gsl/narrow>

#include <opencv2/calib3d.hpp>

#include <spdlog/spdlog.h>

#include "zenslam/utils_slam.h"

namespace zenslam
{
    estimator::estimator(const calibration& calib, const class options::slam& opts)
        : _calibration(calib), _options(opts)
    {
    }

    namespace
    {
        auto median(std::vector<double> v) -> double
        {
            if (v.empty())
                return 0.0;
            std::ranges::nth_element(v, v.begin() + v.size() / 2);
            return v[v.size() / 2];
        }
    }

    auto estimator::estimate_pose_3d2d
    (
        const std::map<size_t, point3d>&  map_points_0,
        const std::map<size_t, keypoint>& map_keypoints_1
    ) const -> pose_data
    {
        std::vector<cv::Point3d> points3d;
        std::vector<cv::Point2d> points2d;
        std::vector<size_t>      indices;
        utils::correspondences_3d2d(map_points_0, map_keypoints_1, points3d, points2d, indices);

        cv::Affine3d pose = cv::Affine3d::Identity();
        auto         out  = zenslam::pose_data { };

        if (points3d.size() >= 6)
        {
            cv::Mat          rvec { pose.rvec() };
            cv::Mat          tvec { pose.translation() };
            std::vector<int> inliers;
            if (cv::solvePnPRansac
                (
                    points3d,
                    points2d,
                    _calibration.camera_matrix[0],
                    cv::Mat(),
                    rvec,
                    tvec,
                    true,
                    1000,
                    gsl::narrow<float>(_options.threshold_3d2d),
                    0.99,
                    inliers
                ))
            {
                out.pose    = cv::Affine3d(rvec, tvec);
                out.indices = indices;

                for (auto i : inliers)
                    out.inliers.push_back(indices[i]);
                auto inliers_set = std::set(out.inliers.begin(), out.inliers.end());
                for (auto i : indices)
                    if (!inliers_set.contains(i))
                        out.outliers.push_back(i);

                std::vector<cv::Point2d> points2d_back;
                cv::projectPoints(points3d, rvec, tvec, _calibration.camera_matrix[0], cv::Mat(), points2d_back);

                for (auto i : inliers)
                {
                    const auto& p = points2d[i];
                    const auto& q = points2d_back[i];
                    out.errors.push_back(cv::norm(p - q));
                }

                return out;
            }

            throw std::runtime_error("SolvePnP failed");
        }

        throw std::runtime_error("Not enough 3D-2D correspondences to compute pose (need > 6)");
    }

    auto estimator::estimate_pose_3d3d
    (
        const std::map<size_t, point3d>& map_points_0,
        const std::map<size_t, point3d>& map_points_1
    ) const -> pose_data
    {
        std::vector<cv::Point3d> points_0;
        std::vector<cv::Point3d> points_1;
        std::vector<size_t>      indices;
        utils::correspondences_3d3d(map_points_0, map_points_1, points_0, points_1, indices);

        if (points_0.size() >= 3)
        {
            cv::Matx33d         R;
            cv::Point3d         t;
            std::vector<size_t> inliers;
            std::vector<size_t> outliers;
            std::vector<double> errors;
            utils::estimate_rigid_ransac(points_0, points_1, R, t, inliers, outliers, errors, _options.threshold_3d3d, 1000);
            return { cv::Affine3d { R, t }, indices, inliers, outliers, errors };
        }

        throw std::runtime_error("Not enough 3D-3D correspondences to compute pose (need > 3)");
    }

    auto estimator::estimate_pose_2d2d
    (
        const std::map<size_t, keypoint>& map_keypoints_0,
        const std::map<size_t, keypoint>& map_keypoints_1,
        const std::map<size_t, point3d>&  map_points3d_0
    ) const -> pose_data
    {
        // Build 2D-2D correspondences (left camera) by matching indices
        std::vector<cv::Point2f> points0;
        std::vector<cv::Point2f> points1;
        std::vector<size_t>      indices;

        points0.reserve(map_keypoints_0.size());
        points1.reserve(map_keypoints_0.size());
        indices.reserve(map_keypoints_0.size());

        for (const auto& [idx, kp0] : map_keypoints_0)
        {
            auto it1 = map_keypoints_1.find(idx);
            if (it1 != map_keypoints_1.end())
            {
                points0.emplace_back(kp0.pt);
                points1.emplace_back(it1->second.pt);
                indices.emplace_back(idx);
            }
        }

        if (points0.size() < 8)
            throw std::runtime_error("Not enough 2D-2D correspondences to compute essential matrix (need >= 8)");

        // Estimate essential matrix and recover R,t (t is unit direction)
        std::vector<uchar> inlier_mask;
        cv::Mat E = cv::findEssentialMat(points0, points1, _calibration.camera_matrix[0], cv::RANSAC, 0.999, _options.epipolar_threshold, inlier_mask);

        if (E.empty())
            throw std::runtime_error("findEssentialMat failed");

        cv::Mat R, t;
        int     inliers = cv::recoverPose(E, points0, points1, _calibration.camera_matrix[0], R, t, inlier_mask);
        if (inliers < 5)
            throw std::runtime_error("recoverPose produced too few inliers");

        // Gather inlier correspondences that also have a previous 3D point for scale estimation
        std::vector<cv::Point2f> inlier_p0;
        std::vector<cv::Point2f> inlier_p1;
        std::vector<cv::Point3d> X0_list;
        std::vector<size_t>      inlier_indices; // global indices of inlier correspondences

        inlier_p0.reserve(inliers);
        inlier_p1.reserve(inliers);
        X0_list.reserve(inliers);

        for (size_t i = 0; i < indices.size(); ++i)
        {
            if (!inlier_mask[i])
                continue;
            const auto idx = indices[i];
            auto       itX = map_points3d_0.find(idx);
            if (itX != map_points3d_0.end())
            {
                inlier_p0.emplace_back(points0[i]);
                inlier_p1.emplace_back(points1[i]);
                X0_list.emplace_back(itX->second);
                inlier_indices.emplace_back(idx);
            }
        }

        if (X0_list.size() < 5)
            throw std::runtime_error("Not enough inliers with previous 3D points to scale 2D-2D pose (need >= 5)");

        // Triangulate up-to-scale points using the recovered R,t (unit) to compute scale
        const cv::Matx33d K(_calibration.camera_matrix[0]);
        const auto Rm = cv::Matx33d(R);
        const auto tv = cv::Vec3d(t);

        cv::Matx34d P0
        (
            K(0, 0),
            K(0, 1),
            K(0, 2),
            0.0,
            K(1, 0),
            K(1, 1),
            K(1, 2),
            0.0,
            K(2, 0),
            K(2, 1),
            K(2, 2),
            0.0
        );

        const cv::Matx33d KR = K * Rm;
        const cv::Vec3d   Kt = K * tv;
        cv::Matx34d       P1
        (
            KR(0, 0),
            KR(0, 1),
            KR(0, 2),
            Kt[0],
            KR(1, 0),
            KR(1, 1),
            KR(1, 2),
            Kt[1],
            KR(2, 0),
            KR(2, 1),
            KR(2, 2),
            Kt[2]
        );

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
            throw std::runtime_error("Unable to compute robust scale for 2D-2D pose");

        const double s = median(scales);

        // Build pose and compute errors as 2D reprojection of X0 into frame 1
        const cv::Point3d t_scaled = cv::Point3d(tv) * s;
        pose_data         out { };
        out.pose    = cv::Affine3d(Rm, t_scaled);
        out.indices = indices; // all correspondences considered

        // Split inliers/outliers
        for (size_t i = 0; i < indices.size(); ++i)
        {
            if (inlier_mask[i])
                out.inliers.push_back(indices[i]);
            else
                out.outliers.push_back(indices[i]);
        }

        // Reprojection error on the subset that has 3D
        std::vector<cv::Point2d> proj;
        cv::Mat                  rvec;
        cv::Rodrigues(R, rvec);
        cv::projectPoints(X0_list, rvec, cv::Mat(t_scaled), _calibration.camera_matrix[0], cv::Mat(), proj);
        for (size_t i = 0; i < proj.size(); ++i)
        {
            out.errors.push_back(cv::norm(proj[i] - cv::Point2d(inlier_p1[i])));
        }

        return out;
    }

    auto estimator::estimate_pose(const std::map<size_t, point3d>& points3d_0, const frame::tracked& tracked_1) const -> estimate_pose_result
    {
        estimate_pose_result result { };

        // Attempt 3D-3D
        try
        {
            result.pose_3d3d = estimate_pose_3d3d(points3d_0, tracked_1.points3d);
        }
        catch (const std::runtime_error& e)
        {
            SPDLOG_WARN("3D-3D pose estimation failed: {}", e.what());
        }

        // Attempt 3D-2D (left camera)
        try
        {
            result.pose_3d2d = estimate_pose_3d2d(points3d_0, tracked_1.keypoints[0]);
        }
        catch (const std::runtime_error& e)
        {
            SPDLOG_WARN("3D-2D pose estimation failed: {}", e.what());
        }

        // Choose the pose with more inliers; if tie, prefer 3D-3D; if both invalid, identity
        const auto inliers_3d3d = result.pose_3d3d.inliers.size();
        const auto inliers_3d2d = result.pose_3d2d.inliers.size();

        // No 2D-2D path in this overload
        if (inliers_3d3d == 0 && inliers_3d2d == 0)
            result.chosen_pose = cv::Affine3d::Identity();
        else if (inliers_3d3d >= inliers_3d2d)
            result.chosen_pose = result.pose_3d3d.pose;
        else
            result.chosen_pose = result.pose_3d2d.pose;

        return result;
    }

    auto estimator::estimate_pose(const frame::estimated& frame_0, const frame::tracked& tracked_1) const -> estimate_pose_result
    {
        estimate_pose_result result { };

        // 3D-3D
        try
        {
            result.pose_3d3d = estimate_pose_3d3d(frame_0.points3d, tracked_1.points3d);
        }
        catch (const std::runtime_error& e)
        {
            SPDLOG_WARN("3D-3D pose estimation failed: {}", e.what());
        }

        // 3D-2D (left camera)
        try
        {
            result.pose_3d2d = estimate_pose_3d2d(frame_0.points3d, tracked_1.keypoints[0]);
        }
        catch (const std::runtime_error& e)
        {
            SPDLOG_WARN("3D-2D pose estimation failed: {}", e.what());
        }

        // 2D-2D with scale from previous triangulated points
        try
        {
            result.pose_2d2d = estimate_pose_2d2d(frame_0.keypoints[0], tracked_1.keypoints[0], frame_0.points3d);
        }
        catch (const std::runtime_error& e)
        {
            SPDLOG_WARN("2D-2D pose estimation failed: {}", e.what());
        }

        const auto in3d3d = result.pose_3d3d.inliers.size();
        const auto in3d2d = result.pose_3d2d.inliers.size();
        const auto in2d2d = result.pose_2d2d.inliers.size();

        // Choose the pose with the most inliers. Tie-breaker: 3D-3D > 3D-2D > 2D-2D
        if (in3d3d == 0 && in3d2d == 0 && in2d2d == 0)
        {
            result.chosen_pose = cv::Affine3d::Identity();
        }
        else
        {
            // Track best by inlier count with tie-breaker preference: 3D-3D > 3D-2D > 2D-2D
            size_t       best_inliers = 0;
            cv::Affine3d best_pose    = cv::Affine3d::Identity();

            if (in3d3d > best_inliers)
            {
                best_inliers = in3d3d;
                if (!result.pose_3d3d.inliers.empty())
                    best_pose = result.pose_3d3d.pose;
            }

            if (in3d2d > best_inliers || (in3d2d == best_inliers && best_pose.matrix == cv::Affine3d::Identity().matrix))
            {
                best_inliers = in3d2d;
                if (!result.pose_3d2d.inliers.empty())
                    best_pose = result.pose_3d2d.pose;
            }

            if (in2d2d > best_inliers || (in2d2d == best_inliers && best_pose.matrix == cv::Affine3d::Identity().matrix))
            {
                best_inliers = in2d2d;
                if (!result.pose_2d2d.inliers.empty())
                    best_pose = result.pose_2d2d.pose;
            }

            result.chosen_count = best_inliers;
            result.chosen_pose  = best_pose;
        }

        return result;
    }
}
