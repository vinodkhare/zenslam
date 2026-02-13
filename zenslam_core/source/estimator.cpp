#include "zenslam/estimator.h"

#include <algorithm>
#include <set>
#include <utility>
#include <numeric>
#include <cmath>

#include <gsl/narrow>

#include <opencv2/calib3d.hpp>

#include <spdlog/spdlog.h>

#include "zenslam/utils_slam.h"
#include "zenslam/utils_std.h"

namespace zenslam
{
    estimator::estimator(calibration calib, slam_options opts) :
        _calibration(std::move(calib)),
        _options(std::move(opts))
    {
    }

    auto estimator::estimate_pose_3d2d(const std::map<size_t, point3d>& map_points_0, const std::map<size_t, keypoint>& map_keypoints_1) const -> pose_data
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
            if (cv::solvePnPRansac(
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
                inliers))
            {
                // Refine pose using Levenberg-Marquardt optimization on inliers
                if (_options.use_pose_refinement && inliers.size() >= 4)
                {
                    std::vector<cv::Point3d> inlier_points3d;
                    std::vector<cv::Point2d> inlier_points2d;
                    inlier_points3d.reserve(inliers.size());
                    inlier_points2d.reserve(inliers.size());
                    
                    for (auto i : inliers)
                    {
                        inlier_points3d.push_back(points3d[i]);
                        inlier_points2d.push_back(points2d[i]);
                    }
                    
                    // Refine using all inliers with LM optimization
                    cv::solvePnPRefineLM(
                        inlier_points3d,
                        inlier_points2d,
                        _calibration.camera_matrix[0],
                        cv::Mat(),
                        rvec,
                        tvec,
                        cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 20, 1e-6));
                }
                
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

    auto estimator::estimate_pose_3d3d(const std::map<size_t, point3d>& map_points_0, const std::map<size_t, point3d>& map_points_1) const -> pose_data
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

    auto estimator::estimate_pose_2d2d(
        const std::map<size_t, keypoint>& map_keypoints_0,
        const std::map<size_t, keypoint>& map_keypoints_1,
        const std::map<size_t, point3d>&  map_points3d_0) const -> pose_data
    {
        // Build 2D-2D correspondences (left camera) by matching indices
        std::vector<cv::Point2f> points0;
        std::vector<cv::Point2f> points1;
        std::vector<size_t>      indices;

        utils::correspondence_2d2d(map_keypoints_0, map_keypoints_1, points0, points1, indices);

        if (points0.size() < 8)
            throw std::runtime_error("Not enough 2D-2D correspondences to compute essential matrix (need >= 8)");

        // Estimate essential matrix and recover R,t (t is unit direction)
        std::vector<uchar> inlier_mask;
        cv::Mat            E = cv::findEssentialMat(points0, points1, _calibration.camera_matrix[0], cv::RANSAC, 0.999, _options.epipolar_threshold, inlier_mask);

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
        const auto        Rm = cv::Matx33d(R);
        const auto        tv = cv::Vec3d(t);

        cv::Matx34d P0(K(0, 0), K(0, 1), K(0, 2), 0.0, K(1, 0), K(1, 1), K(1, 2), 0.0, K(2, 0), K(2, 1), K(2, 2), 0.0);

        const cv::Matx33d KR = K * Rm;
        const cv::Vec3d   Kt = K * tv;
        cv::Matx34d       P1(KR(0, 0), KR(0, 1), KR(0, 2), Kt[0], KR(1, 0), KR(1, 1), KR(1, 2), Kt[1], KR(2, 0), KR(2, 1), KR(2, 2), Kt[2]);

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

        const double s = utils::median(scales);

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

    auto estimator::estimate_pose_3d2d_lines(const std::map<size_t, line3d>& map_lines_0, const std::map<size_t, keyline>& map_keylines_1) const -> pose_data
    {
        std::vector<cv::Point3d> lines3d_p1,    lines3d_p2;
        std::vector<cv::Point2d> keylines2d_p1, keylines2d_p2;
        std::vector<size_t>      indices;
        utils::correspondences_3d2d_lines(map_lines_0, map_keylines_1, lines3d_p1, lines3d_p2, keylines2d_p1, keylines2d_p2, indices);

        cv::Affine3d pose = cv::Affine3d::Identity();
        auto         out  = zenslam::pose_data { };

        if (lines3d_p1.size() >= 3)
        {
            // Combine all endpoints into single vector for PnP
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

            cv::Mat          rvec { pose.rvec() };
            cv::Mat          tvec { pose.translation() };
            std::vector<int> inliers;
            if (cv::solvePnPRansac(
                all_points3d,
                all_points2d,
                _calibration.camera_matrix[0],
                cv::Mat(),
                rvec,
                tvec,
                true,
                1000,
                gsl::narrow<float>(_options.threshold_3d2d),
                0.99,
                inliers))
            {
                // Refine pose using Levenberg-Marquardt optimization on inliers
                if (_options.use_pose_refinement && inliers.size() >= 6)
                {
                    std::vector<cv::Point3d> inlier_points3d;
                    std::vector<cv::Point2d> inlier_points2d;
                    inlier_points3d.reserve(inliers.size());
                    inlier_points2d.reserve(inliers.size());
                    
                    for (auto i : inliers)
                    {
                        inlier_points3d.push_back(all_points3d[i]);
                        inlier_points2d.push_back(all_points2d[i]);
                    }
                    
                    // Refine using all inliers with LM optimization
                    cv::solvePnPRefineLM(
                        inlier_points3d,
                        inlier_points2d,
                        _calibration.camera_matrix[0],
                        cv::Mat(),
                        rvec,
                        tvec,
                        cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 20, 1e-6));
                }
                
                out.pose    = cv::Affine3d(rvec, tvec);
                out.indices = indices;

                // Map endpoint inlier indices back to line indices
                std::set<size_t> unique_line_indices;
                for (auto i : inliers)
                {
                    const size_t line_idx = i / 2;
                    if (line_idx < indices.size())
                        unique_line_indices.insert(indices[line_idx]);
                }

                for (const auto& idx : unique_line_indices)
                    out.inliers.push_back(idx);

                // Outliers
                std::set<size_t> inlier_set(unique_line_indices.begin(), unique_line_indices.end());
                for (const auto& idx : indices)
                {
                    if (!inlier_set.contains(idx))
                        out.outliers.push_back(idx);
                }

                // Reprojection error for each line endpoint
                std::vector<cv::Point2d> proj;
                cv::Mat                  rvec_out;
                cv::Rodrigues(out.pose.rotation(), rvec_out);
                cv::projectPoints(all_points3d, rvec_out, cv::Mat(out.pose.translation()), _calibration.camera_matrix[0], cv::Mat(), proj);
                for (size_t i = 0; i < proj.size(); ++i)
                {
                    out.errors.push_back(cv::norm(proj[i] - all_points2d[i]));
                }

                return out;
            }

            throw std::runtime_error("SolvePnP failed for lines");
        }

        throw std::runtime_error("Not enough 3D-2D line correspondences to compute pose (need >= 3)");
    }

    auto estimator::estimate_pose_3d3d_lines(const std::map<size_t, line3d>& map_lines_0, const std::map<size_t, line3d>& map_lines_1) const -> pose_data
    {
        std::vector<cv::Point3d> lines3d_0_p1, lines3d_0_p2;
        std::vector<cv::Point3d> lines3d_1_p1, lines3d_1_p2;
        std::vector<size_t>      indices;
        utils::correspondences_3d3d_lines(map_lines_0, map_lines_1, lines3d_0_p1, lines3d_0_p2, lines3d_1_p1, lines3d_1_p2, indices);

        if (lines3d_0_p1.size() >= 2)
        {
            // Combine all endpoints into single vectors for RANSAC
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

            cv::Matx33d         R;
            cv::Point3d         t;
            std::vector<size_t> inliers;
            std::vector<size_t> outliers;
            std::vector<double> errors;
            utils::estimate_rigid_ransac(all_points_0, all_points_1, R, t, inliers, outliers, errors, _options.threshold_3d3d, 1000);

            // Map endpoint inlier indices back to line indices
            std::set<size_t> unique_line_inliers;
            for (const auto& idx : inliers)
            {
                const size_t line_idx = idx / 2;
                if (line_idx < indices.size())
                    unique_line_inliers.insert(indices[line_idx]);
            }

            // Map endpoint outlier indices back to line indices
            std::set<size_t> unique_line_outliers;
            for (const auto& idx : outliers)
            {
                const size_t line_idx = idx / 2;
                if (line_idx < indices.size())
                    unique_line_outliers.insert(indices[line_idx]);
            }

            std::vector<size_t> final_inliers(unique_line_inliers.begin(), unique_line_inliers.end());
            std::vector<size_t> final_outliers(unique_line_outliers.begin(), unique_line_outliers.end());

            return { cv::Affine3d { R, t }, indices, final_inliers, final_outliers, errors };
        }

        throw std::runtime_error("Not enough 3D-3D line correspondences to compute pose (need >= 2)");
    }

    auto estimator::estimate_pose_3d2d_combined(
        const std::map<size_t, point3d>& map_points_0,
        const std::map<size_t, keypoint>& map_keypoints_1,
        const std::map<size_t, line3d>& map_lines_0,
        const std::map<size_t, keyline>& map_keylines_1) const -> pose_data
    {
        // Gather point correspondences
        std::vector<cv::Point3d> points3d;
        std::vector<cv::Point2d> points2d;
        std::vector<size_t>      point_indices;
        utils::correspondences_3d2d(map_points_0, map_keypoints_1, points3d, points2d, point_indices);

        // Gather line endpoint correspondences
        std::vector<cv::Point3d> lines3d_p1, lines3d_p2;
        std::vector<cv::Point2d> keylines2d_p1, keylines2d_p2;
        std::vector<size_t>      line_indices;
        utils::correspondences_3d2d_lines(map_lines_0, map_keylines_1, lines3d_p1, lines3d_p2, keylines2d_p1, keylines2d_p2, line_indices);

        // Combine everything into unified vectors for PnP
        std::vector<cv::Point3d> all_points3d;
        std::vector<cv::Point2d> all_points2d;
        all_points3d.reserve(points3d.size() + lines3d_p1.size() * 2);
        all_points2d.reserve(points2d.size() + keylines2d_p1.size() * 2);

        // Add points
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

        cv::Affine3d pose = cv::Affine3d::Identity();
        auto         out  = zenslam::pose_data { };

        const size_t min_correspondences = 6;
        if (all_points3d.size() >= min_correspondences)
        {
            cv::Mat          rvec { pose.rvec() };
            cv::Mat          tvec { pose.translation() };
            std::vector<int> inliers;
            
            if (cv::solvePnPRansac(
                all_points3d,
                all_points2d,
                _calibration.camera_matrix[0],
                cv::Mat(),
                rvec,
                tvec,
                true,
                1000,
                gsl::narrow<float>(_options.threshold_3d2d),
                0.99,
                inliers))
            {
                // Refine pose using Levenberg-Marquardt optimization on inliers
                if (_options.use_pose_refinement && inliers.size() >= 4)
                {
                    std::vector<cv::Point3d> inlier_points3d;
                    std::vector<cv::Point2d> inlier_points2d;
                    inlier_points3d.reserve(inliers.size());
                    inlier_points2d.reserve(inliers.size());
                    
                    for (auto i : inliers)
                    {
                        inlier_points3d.push_back(all_points3d[i]);
                        inlier_points2d.push_back(all_points2d[i]);
                    }
                    
                    // Refine using all inliers with LM optimization
                    cv::solvePnPRefineLM(
                        inlier_points3d,
                        inlier_points2d,
                        _calibration.camera_matrix[0],
                        cv::Mat(),
                        rvec,
                        tvec,
                        cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 20, 1e-6));
                }
                
                out.pose = cv::Affine3d(rvec, tvec);

                // Map inliers back to original feature indices
                // Track which features contributed to the solution
                std::set<size_t> inlier_point_ids;
                std::set<size_t> inlier_line_ids;
                
                for (auto idx : inliers)
                {
                    if (idx < num_points)
                    {
                        // Point inlier
                        inlier_point_ids.insert(point_indices[idx]);
                    }
                    else
                    {
                        // Line endpoint inlier - map back to line ID
                        const size_t line_endpoint_idx = idx - num_points;
                        const size_t line_idx = line_endpoint_idx / 2;
                        if (line_idx < line_indices.size())
                            inlier_line_ids.insert(line_indices[line_idx]);
                    }
                }

                // Combine all unique feature IDs
                out.indices.reserve(point_indices.size() + line_indices.size());
                out.indices.insert(out.indices.end(), point_indices.begin(), point_indices.end());
                out.indices.insert(out.indices.end(), line_indices.begin(), line_indices.end());

                out.inliers.reserve(inlier_point_ids.size() + inlier_line_ids.size());
                out.inliers.insert(out.inliers.end(), inlier_point_ids.begin(), inlier_point_ids.end());
                out.inliers.insert(out.inliers.end(), inlier_line_ids.begin(), inlier_line_ids.end());

                // Outliers are all features not in inlier sets
                for (auto id : point_indices)
                    if (!inlier_point_ids.contains(id))
                        out.outliers.push_back(id);
                for (auto id : line_indices)
                    if (!inlier_line_ids.contains(id))
                        out.outliers.push_back(id);

                // Compute reprojection errors
                std::vector<cv::Point2d> points2d_back;
                cv::projectPoints(all_points3d, rvec, tvec, _calibration.camera_matrix[0], cv::Mat(), points2d_back);
                
                for (auto i : inliers)
                {
                    const auto& p = all_points2d[i];
                    const auto& q = points2d_back[i];
                    out.errors.push_back(cv::norm(p - q));
                }

                SPDLOG_DEBUG("Combined 3D-2D: {} correspondences ({} points + {} lines), {} inliers ({} point + {} line features)",
                    all_points3d.size(), num_points, lines3d_p1.size(), 
                    inliers.size(), inlier_point_ids.size(), inlier_line_ids.size());

                return out;
            }

            throw std::runtime_error("Combined 3D-2D PnP RANSAC failed");
        }

        throw std::runtime_error("Not enough combined 3D-2D correspondences (need >= 6)");
    }

    auto estimator::estimate_pose_3d3d_combined(
        const std::map<size_t, point3d>& map_points_0,
        const std::map<size_t, point3d>& map_points_1,
        const std::map<size_t, line3d>& map_lines_0,
        const std::map<size_t, line3d>& map_lines_1) const -> pose_data
    {
        // Gather point correspondences
        std::vector<cv::Point3d> points_0;
        std::vector<cv::Point3d> points_1;
        std::vector<size_t>      point_indices;
        utils::correspondences_3d3d(map_points_0, map_points_1, points_0, points_1, point_indices);

        // Gather line endpoint correspondences
        std::vector<cv::Point3d> lines3d_0_p1, lines3d_0_p2;
        std::vector<cv::Point3d> lines3d_1_p1, lines3d_1_p2;
        std::vector<size_t>      line_indices;
        utils::correspondences_3d3d_lines(map_lines_0, map_lines_1, lines3d_0_p1, lines3d_0_p2, lines3d_1_p1, lines3d_1_p2, line_indices);

        // Combine everything into unified vectors for rigid transformation estimation
        std::vector<cv::Point3d> all_points_0;
        std::vector<cv::Point3d> all_points_1;
        all_points_0.reserve(points_0.size() + lines3d_0_p1.size() * 2);
        all_points_1.reserve(points_1.size() + lines3d_1_p1.size() * 2);

        // Add points
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

        if (all_points_0.size() >= 3)
        {
            cv::Matx33d         R;
            cv::Point3d         t;
            std::vector<size_t> inliers;
            std::vector<size_t> outliers;
            std::vector<double> errors;
            utils::estimate_rigid_ransac(all_points_0, all_points_1, R, t, inliers, outliers, errors, _options.threshold_3d3d, 1000);

            // Map inliers back to original feature indices
            std::set<size_t> inlier_point_ids;
            std::set<size_t> inlier_line_ids;
            
            for (auto idx : inliers)
            {
                if (idx < num_points)
                {
                    // Point inlier
                    inlier_point_ids.insert(point_indices[idx]);
                }
                else
                {
                    // Line endpoint inlier - map back to line ID
                    const size_t line_endpoint_idx = idx - num_points;
                    const size_t line_idx = line_endpoint_idx / 2;
                    if (line_idx < line_indices.size())
                        inlier_line_ids.insert(line_indices[line_idx]);
                }
            }

            // Map outliers similarly
            std::set<size_t> outlier_point_ids;
            std::set<size_t> outlier_line_ids;
            
            for (auto idx : outliers)
            {
                if (idx < num_points)
                {
                    outlier_point_ids.insert(point_indices[idx]);
                }
                else
                {
                    const size_t line_endpoint_idx = idx - num_points;
                    const size_t line_idx = line_endpoint_idx / 2;
                    if (line_idx < line_indices.size())
                        outlier_line_ids.insert(line_indices[line_idx]);
                }
            }

            pose_data out;
            out.pose = cv::Affine3d { R, t };

            // Combine all unique feature IDs
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

            SPDLOG_DEBUG("Combined 3D-3D: {} correspondences ({} points + {} lines), {} inliers ({} point + {} line features)",
                all_points_0.size(), num_points, lines3d_0_p1.size(), 
                inliers.size(), inlier_point_ids.size(), inlier_line_ids.size());

            return out;
        }

        throw std::runtime_error("Not enough combined 3D-3D correspondences (need >= 3)");
    }

    auto estimator::estimate_pose_2d2d_combined(
        const std::map<size_t, keypoint>& map_keypoints_0,
        const std::map<size_t, keypoint>& map_keypoints_1,
        const std::map<size_t, point3d>& map_points3d_0,
        const std::map<size_t, line3d>& map_lines_0,
        const std::map<size_t, keyline>& map_keylines_0,
        const std::map<size_t, keyline>& map_keylines_1) const -> pose_data
    {
        // Gather point correspondences
        std::vector<cv::Point2f> points0;
        std::vector<cv::Point2f> points1;
        std::vector<size_t>      point_indices;
        utils::correspondence_2d2d(map_keypoints_0, map_keypoints_1, points0, points1, point_indices);

        // Gather line endpoint correspondences
        std::vector<cv::Point2f> keylines0_p1, keylines0_p2;
        std::vector<cv::Point2f> keylines1_p1, keylines1_p2;
        std::vector<size_t>      line_indices;
        
        // Extract line correspondences manually (similar to points)
        for (const auto& [id0, kl0] : map_keylines_0)
        {
            auto it1 = map_keylines_1.find(id0);
            if (it1 != map_keylines_1.end())
            {
                const auto& kl1 = it1->second;
                keylines0_p1.emplace_back(kl0.startPointX, kl0.startPointY);
                keylines0_p2.emplace_back(kl0.endPointX, kl0.endPointY);
                keylines1_p1.emplace_back(kl1.startPointX, kl1.startPointY);
                keylines1_p2.emplace_back(kl1.endPointX, kl1.endPointY);
                line_indices.push_back(id0);
            }
        }

        // Combine into unified vectors
        std::vector<cv::Point2f> all_points0;
        std::vector<cv::Point2f> all_points1;
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
            throw std::runtime_error("Not enough combined 2D-2D correspondences to compute essential matrix (need >= 8)");

        // Estimate essential matrix and recover R,t
        std::vector<uchar> inlier_mask;
        cv::Mat E = cv::findEssentialMat(all_points0, all_points1, _calibration.camera_matrix[0], cv::RANSAC, 0.999, _options.epipolar_threshold, inlier_mask);

        if (E.empty())
            throw std::runtime_error("findEssentialMat failed");

        cv::Mat R, t;
        int inliers = cv::recoverPose(E, all_points0, all_points1, _calibration.camera_matrix[0], R, t, inlier_mask);
        if (inliers < 5)
            throw std::runtime_error("recoverPose produced too few inliers");

        // Gather inliers that have 3D data for scale estimation
        std::vector<cv::Point2f> inlier_p0;
        std::vector<cv::Point2f> inlier_p1;
        std::vector<cv::Point3d> X0_list;

        for (size_t i = 0; i < all_points0.size(); ++i)
        {
            if (!inlier_mask[i])
                continue;

            cv::Point3d X0;
            bool has_3d = false;

            if (i < num_points)
            {
                // Point inlier - check if has 3D
                auto itX = map_points3d_0.find(point_indices[i]);
                if (itX != map_points3d_0.end())
                {
                    X0 = itX->second;
                    has_3d = true;
                }
            }
            else
            {
                // Line endpoint inlier - check if line has 3D
                const size_t line_endpoint_idx = i - num_points;
                const size_t line_idx = line_endpoint_idx / 2;
                const bool is_endpoint1 = (line_endpoint_idx % 2) == 0;
                
                if (line_idx < line_indices.size())
                {
                    auto itL = map_lines_0.find(line_indices[line_idx]);
                    if (itL != map_lines_0.end())
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
            throw std::runtime_error("Not enough inliers with previous 3D data to scale combined 2D-2D pose (need >= 5)");

        // Triangulate and compute scale
        const cv::Matx33d K(_calibration.camera_matrix[0]);
        const auto Rm = cv::Matx33d(R);
        const auto tv = cv::Vec3d(t);

        cv::Matx34d P0(K(0,0), K(0,1), K(0,2), 0.0, K(1,0), K(1,1), K(1,2), 0.0, K(2,0), K(2,1), K(2,2), 0.0);
        const cv::Matx33d KR = K * Rm;
        const cv::Vec3d Kt = K * tv;
        cv::Matx34d P1(KR(0,0), KR(0,1), KR(0,2), Kt[0], KR(1,0), KR(1,1), KR(1,2), Kt[1], KR(2,0), KR(2,1), KR(2,2), Kt[2]);

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
            throw std::runtime_error("Unable to compute robust scale for combined 2D-2D pose");

        const double s = utils::median(scales);
        const cv::Point3d t_scaled = cv::Point3d(tv) * s;

        // Build output
        pose_data out;
        out.pose = cv::Affine3d(Rm, t_scaled);

        // Map inliers/outliers back to feature IDs
        std::set<size_t> inlier_point_ids;
        std::set<size_t> inlier_line_ids;

        for (size_t i = 0; i < all_points0.size(); ++i)
        {
            if (!inlier_mask[i])
                continue;

            if (i < num_points)
            {
                inlier_point_ids.insert(point_indices[i]);
            }
            else
            {
                const size_t line_endpoint_idx = i - num_points;
                const size_t line_idx = line_endpoint_idx / 2;
                if (line_idx < line_indices.size())
                    inlier_line_ids.insert(line_indices[line_idx]);
            }
        }

        // Combine indices
        out.indices.reserve(point_indices.size() + line_indices.size());
        out.indices.insert(out.indices.end(), point_indices.begin(), point_indices.end());
        out.indices.insert(out.indices.end(), line_indices.begin(), line_indices.end());

        out.inliers.reserve(inlier_point_ids.size() + inlier_line_ids.size());
        out.inliers.insert(out.inliers.end(), inlier_point_ids.begin(), inlier_point_ids.end());
        out.inliers.insert(out.inliers.end(), inlier_line_ids.begin(), inlier_line_ids.end());

        // Outliers are features not in inlier sets
        for (auto id : point_indices)
            if (!inlier_point_ids.contains(id))
                out.outliers.push_back(id);
        for (auto id : line_indices)
            if (!inlier_line_ids.contains(id))
                out.outliers.push_back(id);

        // Compute reprojection errors
        std::vector<cv::Point2d> proj;
        cv::Mat rvec;
        cv::Rodrigues(R, rvec);
        cv::projectPoints(X0_list, rvec, cv::Mat(t_scaled), _calibration.camera_matrix[0], cv::Mat(), proj);
        for (size_t i = 0; i < proj.size(); ++i)
        {
            out.errors.push_back(cv::norm(proj[i] - cv::Point2d(inlier_p1[i])));
        }

        SPDLOG_DEBUG("Combined 2D-2D: {} correspondences ({} points + {} lines), {} inliers ({} point + {} line features)",
            all_points0.size(), num_points, keylines0_p1.size(), 
            inliers, inlier_point_ids.size(), inlier_line_ids.size());

        return out;
    }

    auto estimator::estimate_pose(const std::map<size_t, point3d>& points3d_0, const frame::tracked& tracked_1) const -> estimate_pose_result
    {
        estimate_pose_result result { };

        // Combined 3D-3D estimation using both points and lines
        // Note: tracked_1.lines3d are already in 3D
        try
        {
            result.pose_3d3d = estimate_pose_3d3d_combined(
                points3d_0,
                tracked_1.points3d,
                std::map<size_t, line3d>{},  // No previous lines in this overload
                tracked_1.lines3d);
            // Store same result in pose_3d3d_lines for backward compatibility
            result.pose_3d3d_lines = result.pose_3d3d;
        }
        catch (const std::runtime_error& e)
        {
            SPDLOG_WARN("Combined 3D-3D pose estimation failed: {}", e.what());
        }

        // Combined 3D-2D estimation using both points and lines
        // Note: tracked_1.lines3d are already in 3D
        try
        {
            result.pose_3d2d = estimate_pose_3d2d_combined(
                points3d_0, 
                tracked_1.keypoints[0],
                tracked_1.lines3d,  // Using current frame's 3D lines
                tracked_1.keylines[0]);
            // Store same result in pose_3d2d_lines for backward compatibility
            result.pose_3d2d_lines = result.pose_3d2d;
        }
        catch (const std::runtime_error& e)
        {
            SPDLOG_WARN("Combined 3D-2D pose estimation failed: {}", e.what());
        }

        // Choose the pose with more inliers; if tied, prefer 3D-3D; if both invalid, identity
        const auto inliers_3d3d       = result.pose_3d3d.inliers.size();
        const auto inliers_3d2d       = result.pose_3d2d.inliers.size();
        const auto inliers_3d2d_lines = result.pose_3d2d_lines.inliers.size();

        // No 2D-2D path or 3D-3D lines in this overload
        size_t       best_inliers = 0;
        cv::Affine3d best_pose    = cv::Affine3d::Identity();

        if (inliers_3d3d > best_inliers)
        {
            best_inliers = inliers_3d3d;
            best_pose    = result.pose_3d3d.pose;
        }

        if (inliers_3d2d > best_inliers)
        {
            best_inliers = inliers_3d2d;
            best_pose    = result.pose_3d2d.pose;
        }

        if (inliers_3d2d_lines > best_inliers)
        {
            best_inliers = inliers_3d2d_lines;
            best_pose    = result.pose_3d2d_lines.pose;
        }

        result.chosen_count = best_inliers;
        result.chosen_pose  = best_pose;

        return result;
    }

    auto estimator::estimate_pose(const frame::estimated& frame_0, const frame::tracked& tracked_1) const -> estimate_pose_result
    {
        estimate_pose_result result { };

        // Combined 3D-3D estimation using both points and lines
        try
        {
            result.pose_3d3d = estimate_pose_3d3d_combined(
                frame_0.points3d,
                tracked_1.points3d,
                frame_0.lines3d,
                tracked_1.lines3d);
            // Store same result in pose_3d3d_lines for backward compatibility
            result.pose_3d3d_lines = result.pose_3d3d;
        }
        catch (const std::runtime_error& e)
        {
            SPDLOG_WARN("Combined 3D-3D pose estimation failed: {}", e.what());
        }

        // Combined 3D-2D estimation using both points and lines
        try
        {
            result.pose_3d2d = estimate_pose_3d2d_combined(
                frame_0.points3d, 
                tracked_1.keypoints[0],
                frame_0.lines3d,
                tracked_1.keylines[0]);
            // Store same result in pose_3d2d_lines for backward compatibility
            result.pose_3d2d_lines = result.pose_3d2d;
        }
        catch (const std::runtime_error& e)
        {
            SPDLOG_WARN("Combined 3D-2D pose estimation failed: {}", e.what());
        }

        // Combined 2D-2D estimation using both points and lines
        try
        {
            result.pose_2d2d = estimate_pose_2d2d_combined(
                frame_0.keypoints[0],
                tracked_1.keypoints[0],
                frame_0.points3d,
                frame_0.lines3d,
                frame_0.keylines[0],
                tracked_1.keylines[0]);
        }
        catch (const std::runtime_error& e)
        {
            SPDLOG_WARN("Combined 2D-2D pose estimation failed: {}", e.what());
        }

        const auto in3d3d       = result.pose_3d3d.inliers.size();
        const auto in3d2d       = result.pose_3d2d.inliers.size();
        const auto in2d2d       = result.pose_2d2d.inliers.size();
        const auto in3d3d_lines = result.pose_3d3d_lines.inliers.size();
        const auto in3d2d_lines = result.pose_3d2d_lines.inliers.size();

        // Choose the pose with the most inliers. Tie-breaker: 3D-3D > 3D-2D > 3D-3D-lines > 3D-2D-lines > 2D-2D
        if (in3d3d == 0 && in3d2d == 0 && in2d2d == 0 && in3d3d_lines == 0 && in3d2d_lines == 0)
        {
            result.chosen_pose = cv::Affine3d::Identity();
        }
        else
        {
            // Track best by inlier count with tie-breaker preference
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

            if (in3d3d_lines > best_inliers || (in3d3d_lines == best_inliers && best_pose.matrix == cv::Affine3d::Identity().matrix))
            {
                best_inliers = in3d3d_lines;
                if (!result.pose_3d3d_lines.inliers.empty())
                    best_pose = result.pose_3d3d_lines.pose;
            }

            if (in3d2d_lines > best_inliers || (in3d2d_lines == best_inliers && best_pose.matrix == cv::Affine3d::Identity().matrix))
            {
                best_inliers = in3d2d_lines;
                if (!result.pose_3d2d_lines.inliers.empty())
                    best_pose = result.pose_3d2d_lines.pose;
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

    /** Helper: Compute confidence weight for a single pose estimate
     *
     * Weight prioritizes:
     * 1. Absolute inlier count (must have meaningful data)
     * 2. Inlier ratio (consistency quality)
     * 3. Error magnitude (accuracy)
     *
     * @param pose_data The pose estimate with inliers/outliers/errors
     * @param method_type "3d3d", "3d2d", "2d2d", "3d3d_lines", "3d2d_lines"
     * @return Confidence weight in range [0, 1]
     */
    static auto compute_pose_weight(const pose_data& pose, const std::string& method_type) -> double
    {
        // Must have meaningful data
        const size_t inliers = pose.inliers.size();
        if (inliers < 3)  // Minimum 3 inliers needed
            return 0.0;

        const size_t total_corr = pose.indices.size();
        if (total_corr == 0)
            return 0.0;

        // Inlier ratio: strict requirement
        const double inlier_ratio = static_cast<double>(inliers) / static_cast<double>(total_corr);
        
        // Penalize low inlier ratios heavily - quality matters
        if (inlier_ratio < 0.3)
            return 0.01 * inlier_ratio;  // Very low confidence if ratio too poor
        
        // Error quality component: lower error = higher quality
        double error_quality = 1.0;
        if (!pose.errors.empty())
        {
            const double mean_error = utils::mean(pose.errors);
            const double std_error  = utils::std_dev(pose.errors);
            
            // Check for outlier errors (indicates bad estimates)
            double error_threshold = mean_error + 2.0 * std_error;
            size_t good_errors = 0;
            for (const auto& err : pose.errors)
            {
                if (err <= error_threshold)
                    good_errors++;
            }
            
            // If too many outlier errors, reduce confidence
            double error_consistency = static_cast<double>(good_errors) / pose.errors.size();
            if (error_consistency < 0.7)
                error_quality *= 0.5;
            
            // Error scale: tuned for typical SLAM reprojection errors
            const double error_scale = (method_type == "3d3d" || method_type == "3d3d_lines") ? 0.1 : 3.0;
            error_quality *= std::exp(-mean_error / error_scale);
        }

        // Feature type reliability: points more reliable than lines
        double type_weight = 1.0;
        if (method_type.find("lines") != std::string::npos)
            type_weight = 0.9;  // Lines only slightly less (was 0.8)
        
        // Scale by absolute inlier count (prefer methods with more data)
        double inlier_boost = std::min(1.0, static_cast<double>(inliers) / 50.0);

        // Combine: inlier_ratio (0.4) + error_quality (0.4) + inlier_boost (0.2)
        const double confidence = (inlier_ratio * 0.4 + error_quality * 0.4 + inlier_boost * 0.2) * type_weight;

        return std::max(0.0, std::min(1.0, confidence));
    }

    /** Helper: Compute covariance uncertainty for a single pose estimate
     *
     * Uncertainty is based on:
     * 1. Error magnitude and consistency (higher error = higher uncertainty)
     * 2. Number of inliers (fewer inliers = higher uncertainty)
     * 3. Inlier ratio (more outliers = higher uncertainty)
     *
     * @param pose_data The pose estimate with errors and inliers
     * @param method_weight The weight assigned to this method (0-1)
     * @param method_type Method identifier for scale adjustment
     * @return Pair of (translation_std, rotation_std) in meters and radians
     */
    static auto compute_pose_covariance(const pose_data& pose, double method_weight, const std::string& method_type) 
        -> std::pair<double, double>
    {
        const size_t inliers = pose.inliers.size();
        const size_t total_corr = pose.indices.size();
        
        // No valid pose = maximum uncertainty
        if (inliers < 3 || total_corr == 0 || method_weight < 1e-6)
            return {10.0, 0.5};  // Large uncertainty
        
        // Base uncertainty from inlier ratio
        // More inliers = lower uncertainty (inverse relationship)
        const double inlier_ratio = static_cast<double>(inliers) / static_cast<double>(total_corr);
        const double ratio_uncertainty = (1.0 - inlier_ratio) / std::sqrt(inlier_ratio);
        
        // Error-based uncertainty
        double error_std = 0.0;
        if (pose.errors.size() > 2)
        {
            double mean_error = utils::mean(pose.errors);
            error_std = utils::std_dev(pose.errors);
            
            // For 3D methods, errors are in meters; for 2D, in pixels
            // Normalize to meters equivalent
            if (method_type == "3d3d" || method_type == "3d3d_lines")
            {
                // Already in meters
            }
            else
            {
                // 2D reprojection error in pixels -> estimate 3D uncertainty
                // Typical: 1 pixel ≈ 0.01 m at 1m depth (depends on focal length)
                const double pixel_to_meter = 0.01;
                mean_error *= pixel_to_meter;
                error_std *= pixel_to_meter;
            }
        }
        
        // Combined translation uncertainty (meters)
        // Combines error magnitude with sampling uncertainty
        double trans_std = std::sqrt(error_std * error_std + ratio_uncertainty * ratio_uncertainty);
        
        // Rotation uncertainty (radians)
        // Typically much smaller than translation
        // Depends on 3D error magnitude
        double rot_std = 0.1 * trans_std;  // Rough scaling: 1cm translation ≈ 1mrad rotation
        
        // Scale by method weight and confidence (less confident = more uncertainty)
        trans_std /= (method_weight + 0.1);  // Avoid division by zero
        rot_std /= (method_weight + 0.1);
        
        // Clamp to reasonable ranges
        trans_std = std::min(5.0, std::max(0.001, trans_std));
        rot_std = std::min(1.0, std::max(0.0001, rot_std));
        
        return {trans_std, rot_std};
    }

    auto estimator::estimate_pose_weighted(const estimate_pose_result& result) const -> weighted_pose_result
    {
        weighted_pose_result fused{};

        // Compute individual weights
        const double w_3d3d       = compute_pose_weight(result.pose_3d3d, "3d3d");
        const double w_3d2d       = compute_pose_weight(result.pose_3d2d, "3d2d");
        const double w_2d2d       = compute_pose_weight(result.pose_2d2d, "2d2d");
        const double w_3d3d_lines = compute_pose_weight(result.pose_3d3d_lines, "3d3d_lines");
        const double w_3d2d_lines = compute_pose_weight(result.pose_3d2d_lines, "3d2d_lines");

        // Sum and normalize weights
        const double weight_sum = w_3d3d + w_3d2d + w_2d2d + w_3d3d_lines + w_3d2d_lines;

        // If no valid poses, return identity
        if (weight_sum < 1e-9)
        {
            fused.pose       = cv::Affine3d::Identity();
            fused.confidence = 0.0;
            fused.best_method = "none";
            return fused;
        }

        // Normalize to sum to 1.0
        fused.weight_3d3d       = w_3d3d / weight_sum;
        fused.weight_3d2d       = w_3d2d / weight_sum;
        fused.weight_2d2d       = w_2d2d / weight_sum;
        fused.weight_3d3d_lines = w_3d3d_lines / weight_sum;
        fused.weight_3d2d_lines = w_3d2d_lines / weight_sum;

        // Find best contributing method
        fused.best_method_inliers = result.chosen_count;  // From original selection
        if (fused.weight_3d3d >= fused.weight_3d2d && fused.weight_3d3d >= fused.weight_2d2d && 
            fused.weight_3d3d >= fused.weight_3d3d_lines && fused.weight_3d3d >= fused.weight_3d2d_lines)
        {
            fused.best_method = "3D-3D Points";
            fused.best_method_inliers = result.pose_3d3d.inliers.size();
        }
        else if (fused.weight_3d2d >= fused.weight_2d2d && fused.weight_3d2d >= fused.weight_3d3d_lines && 
                 fused.weight_3d2d >= fused.weight_3d2d_lines)
        {
            fused.best_method = "3D-2D Points";
            fused.best_method_inliers = result.pose_3d2d.inliers.size();
        }
        else if (fused.weight_2d2d >= fused.weight_3d3d_lines && fused.weight_2d2d >= fused.weight_3d2d_lines)
        {
            fused.best_method = "2D-2D Points";
            fused.best_method_inliers = result.pose_2d2d.inliers.size();
        }
        else if (fused.weight_3d3d_lines >= fused.weight_3d2d_lines)
        {
            fused.best_method = "3D-3D Lines";
            fused.best_method_inliers = result.pose_3d3d_lines.inliers.size();
        }
        else
        {
            fused.best_method = "3D-2D Lines";
            fused.best_method_inliers = result.pose_3d2d_lines.inliers.size();
        }

        // Fuse translation: weighted average
        cv::Vec3d fused_translation = cv::Vec3d::zeros();
        if (fused.weight_3d3d > 0) fused_translation += result.pose_3d3d.pose.translation() * fused.weight_3d3d;
        if (fused.weight_3d2d > 0) fused_translation += result.pose_3d2d.pose.translation() * fused.weight_3d2d;
        if (fused.weight_2d2d > 0) fused_translation += result.pose_2d2d.pose.translation() * fused.weight_2d2d;
        if (fused.weight_3d3d_lines > 0) fused_translation += result.pose_3d3d_lines.pose.translation() * fused.weight_3d3d_lines;
        if (fused.weight_3d2d_lines > 0) fused_translation += result.pose_3d2d_lines.pose.translation() * fused.weight_3d2d_lines;

        // For rotation fusion, use weighted selection instead of averaging
        // (Rotation vector averaging is mathematically problematic)
        // Select rotation from method with highest weight
        cv::Matx33d fused_rotmat = cv::Matx33d::eye();
        
        if (fused.weight_3d3d > fused.weight_3d2d && fused.weight_3d3d > fused.weight_2d2d &&
            fused.weight_3d3d > fused.weight_3d3d_lines && fused.weight_3d3d > fused.weight_3d2d_lines)
        {
            fused_rotmat = result.pose_3d3d.pose.rotation();
        }
        else if (fused.weight_3d2d > fused.weight_2d2d && fused.weight_3d2d > fused.weight_3d3d_lines && 
                 fused.weight_3d2d > fused.weight_3d2d_lines)
        {
            fused_rotmat = result.pose_3d2d.pose.rotation();
        }
        else if (fused.weight_2d2d > fused.weight_3d3d_lines && fused.weight_2d2d > fused.weight_3d2d_lines)
        {
            fused_rotmat = result.pose_2d2d.pose.rotation();
        }
        else if (fused.weight_3d3d_lines > fused.weight_3d2d_lines)
        {
            fused_rotmat = result.pose_3d3d_lines.pose.rotation();
        }
        else if (fused.weight_3d2d_lines > 0.0)
        {
            fused_rotmat = result.pose_3d2d_lines.pose.rotation();
        }

        // Construct fused pose: best rotation + weighted average translation
        fused.pose = cv::Affine3d(fused_rotmat, fused_translation);

        // Compute overall confidence as weighted average of individual confidences
        fused.confidence = fused.weight_3d3d * w_3d3d +
                          fused.weight_3d2d * w_3d2d +
                          fused.weight_2d2d * w_2d2d +
                          fused.weight_3d3d_lines * w_3d3d_lines +
                          fused.weight_3d2d_lines * w_3d2d_lines;

        // Count total inliers across all methods
        fused.total_inliers = result.pose_3d3d.inliers.size() +
                             result.pose_3d2d.inliers.size() +
                             result.pose_2d2d.inliers.size() +
                             result.pose_3d3d_lines.inliers.size() +
                             result.pose_3d2d_lines.inliers.size();

        SPDLOG_TRACE("Weighted pose fusion details:");
        SPDLOG_TRACE("  3D-3D Points:   inliers={}, ratio={:.2f}, weight={:.3f}",
            result.pose_3d3d.inliers.size(),
            result.pose_3d3d.indices.empty() ? 0.0 : static_cast<double>(result.pose_3d3d.inliers.size()) / result.pose_3d3d.indices.size(),
            fused.weight_3d3d);
        SPDLOG_TRACE("  3D-2D Points:   inliers={}, ratio={:.2f}, weight={:.3f}",
            result.pose_3d2d.inliers.size(),
            result.pose_3d2d.indices.empty() ? 0.0 : static_cast<double>(result.pose_3d2d.inliers.size()) / result.pose_3d2d.indices.size(),
            fused.weight_3d2d);
        SPDLOG_TRACE("  2D-2D Points:   inliers={}, ratio={:.2f}, weight={:.3f}",
            result.pose_2d2d.inliers.size(),
            result.pose_2d2d.indices.empty() ? 0.0 : static_cast<double>(result.pose_2d2d.inliers.size()) / result.pose_2d2d.indices.size(),
            fused.weight_2d2d);
        SPDLOG_TRACE("  3D-3D Lines:    inliers={}, ratio={:.2f}, weight={:.3f}",
            result.pose_3d3d_lines.inliers.size(),
            result.pose_3d3d_lines.indices.empty() ? 0.0 : static_cast<double>(result.pose_3d3d_lines.inliers.size()) / result.pose_3d3d_lines.indices.size(),
            fused.weight_3d3d_lines);
        SPDLOG_TRACE("  3D-2D Lines:    inliers={}, ratio={:.2f}, weight={:.3f}",
            result.pose_3d2d_lines.inliers.size(),
            result.pose_3d2d_lines.indices.empty() ? 0.0 : static_cast<double>(result.pose_3d2d_lines.inliers.size()) / result.pose_3d2d_lines.indices.size(),
            fused.weight_3d2d_lines);
        
        SPDLOG_DEBUG("Pose fusion: best={} (inliers={}, weight={:.3f}), confidence={:.3f}, total_inliers={}",
            fused.best_method, fused.best_method_inliers, 
            fused.weight_3d3d > fused.weight_3d2d ? fused.weight_3d3d :
            fused.weight_3d2d > fused.weight_2d2d ? fused.weight_3d2d :
            fused.weight_2d2d > fused.weight_3d3d_lines ? fused.weight_2d2d :
            fused.weight_3d3d_lines > fused.weight_3d2d_lines ? fused.weight_3d3d_lines : fused.weight_3d2d_lines,
            fused.confidence, fused.total_inliers);

        // === Compute covariance from all methods ===
        // Weighted combination of uncertainty estimates
        const auto [cov_3d3d_t, cov_3d3d_r]       = compute_pose_covariance(result.pose_3d3d, fused.weight_3d3d, "3d3d");
        const auto [cov_3d2d_t, cov_3d2d_r]       = compute_pose_covariance(result.pose_3d2d, fused.weight_3d2d, "3d2d");
        const auto [cov_2d2d_t, cov_2d2d_r]       = compute_pose_covariance(result.pose_2d2d, fused.weight_2d2d, "2d2d");
        const auto [cov_3d3d_lines_t, cov_3d3d_lines_r] = compute_pose_covariance(result.pose_3d3d_lines, fused.weight_3d3d_lines, "3d3d_lines");
        const auto [cov_3d2d_lines_t, cov_3d2d_lines_r] = compute_pose_covariance(result.pose_3d2d_lines, fused.weight_3d2d_lines, "3d2d_lines");

        // Weighted average of uncertainties
        fused.translation_std = fused.weight_3d3d * cov_3d3d_t +
                               fused.weight_3d2d * cov_3d2d_t +
                               fused.weight_2d2d * cov_2d2d_t +
                               fused.weight_3d3d_lines * cov_3d3d_lines_t +
                               fused.weight_3d2d_lines * cov_3d2d_lines_t;

        fused.rotation_std = fused.weight_3d3d * cov_3d3d_r +
                            fused.weight_3d2d * cov_3d2d_r +
                            fused.weight_2d2d * cov_2d2d_r +
                            fused.weight_3d3d_lines * cov_3d3d_lines_r +
                            fused.weight_3d2d_lines * cov_3d2d_lines_r;

        // Build diagonal 6x6 covariance matrix (simplified: assumes independent x,y,z and roll,pitch,yaw)
        fused.pose_covariance = cv::Matx66d::zeros();
        const double trans_var = fused.translation_std * fused.translation_std;
        const double rot_var = fused.rotation_std * fused.rotation_std;
        
        // Translation components (x, y, z)
        fused.pose_covariance(0, 0) = trans_var;
        fused.pose_covariance(1, 1) = trans_var;
        fused.pose_covariance(2, 2) = trans_var;
        
        // Rotation components (roll, pitch, yaw)
        fused.pose_covariance(3, 3) = rot_var;
        fused.pose_covariance(4, 4) = rot_var;
        fused.pose_covariance(5, 5) = rot_var;
        
        fused.has_valid_covariance = (fused.total_inliers >= 5);

        SPDLOG_DEBUG("Pose covariance: translation_std={:.4f}m, rotation_std={:.6f}rad ({:.4f}deg)",
            fused.translation_std, fused.rotation_std, fused.rotation_std * 180.0 / M_PI);

        return fused;
    }} // namespace zenslam