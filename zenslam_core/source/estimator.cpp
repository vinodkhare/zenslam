#include "zenslam/estimator.h"

#include <algorithm>
#include <set>
#include <utility>

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

    auto estimator::estimate_pose(const std::map<size_t, point3d>& points3d_0, const frame::tracked& tracked_1) const -> estimate_pose_result
    {
        estimate_pose_result result { };

        // Attempt 3D-3D points
        try
        {
            result.pose_3d3d = estimate_pose_3d3d(points3d_0, tracked_1.points3d);
        }
        catch (const std::runtime_error& e)
        {
            SPDLOG_WARN("3D-3D point pose estimation failed: {}", e.what());
        }

        // Attempt 3D-2D points
        try
        {
            result.pose_3d2d = estimate_pose_3d2d(points3d_0, tracked_1.keypoints[0]);
        }
        catch (const std::runtime_error& e)
        {
            SPDLOG_WARN("3D-2D point pose estimation failed: {}", e.what());
        }

        // Attempt 3D-2D lines (no previous 3D lines available in this overload)
        try
        {
            result.pose_3d2d_lines = estimate_pose_3d2d_lines(tracked_1.lines3d, tracked_1.keylines[0]);
        }
        catch (const std::runtime_error& e)
        {
            SPDLOG_WARN("3D-2D line pose estimation failed: {}", e.what());
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

        // 3D-3D points
        try
        {
            result.pose_3d3d = estimate_pose_3d3d(frame_0.points3d, tracked_1.points3d);
        }
        catch (const std::runtime_error& e)
        {
            SPDLOG_WARN("3D-3D point pose estimation failed: {}", e.what());
        }

        // 3D-2D points (left camera)
        try
        {
            result.pose_3d2d = estimate_pose_3d2d(frame_0.points3d, tracked_1.keypoints[0]);
        }
        catch (const std::runtime_error& e)
        {
            SPDLOG_WARN("3D-2D point pose estimation failed: {}", e.what());
        }

        // 2D-2D points with scale from previous triangulated points
        try
        {
            result.pose_2d2d = estimate_pose_2d2d(frame_0.keypoints[0], tracked_1.keypoints[0], frame_0.points3d);
        }
        catch (const std::runtime_error& e)
        {
            SPDLOG_WARN("2D-2D point pose estimation failed: {}", e.what());
        }

        // 3D-3D lines
        try
        {
            result.pose_3d3d_lines = estimate_pose_3d3d_lines(frame_0.lines3d, tracked_1.lines3d);
        }
        catch (const std::runtime_error& e)
        {
            SPDLOG_WARN("3D-3D line pose estimation failed: {}", e.what());
        }

        // 3D-2D lines (left camera)
        try
        {
            result.pose_3d2d_lines = estimate_pose_3d2d_lines(frame_0.lines3d, tracked_1.keylines[0]);
        }
        catch (const std::runtime_error& e)
        {
            SPDLOG_WARN("3D-2D line pose estimation failed: {}", e.what());
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
} // namespace zenslam
