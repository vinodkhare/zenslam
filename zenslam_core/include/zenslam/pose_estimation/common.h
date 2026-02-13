#pragma once

#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

namespace zenslam
{
    class slam_options;
}

namespace zenslam::pose_estimation
{

    /// Result of PnP estimation with optional refinement
    struct pnp_result
    {
        cv::Mat rvec;
        cv::Mat tvec;
        std::vector<int> inliers;
        bool success{ false };
    };

    /// Performs PnP RANSAC with optional Levenberg-Marquardt refinement
    /// Uses modern C++23 structured bindings and cleaner parameter passing
    [[nodiscard]] inline auto solve_pnp_ransac(
        const std::vector<cv::Point3d>& points3d,
        const std::vector<cv::Point2d>& points2d,
        const cv::Mat& camera_matrix,
        const slam_options& options) -> pnp_result
    {
        if (points3d.size() < 6)
            return {};

        pnp_result result;
        result.rvec = cv::Mat::zeros(3, 1, CV_64F);
        result.tvec = cv::Mat::zeros(3, 1, CV_64F);

        result.success = cv::solvePnPRansac(
            points3d,
            points2d,
            camera_matrix,
            cv::Mat(),
            result.rvec,
            result.tvec,
            false,
            options.pnp->iterations,
            options.pnp->threshold,
            options.pnp->confidence,
            result.inliers);

        if (!result.success)
            return result;

        // Refine using inliers with LM optimization
        if (options.pnp->use_refinement && result.inliers.size() >= static_cast<size_t>(options.pnp->min_refinement_inliers))
        {
            std::vector<cv::Point3d> inlier_points3d;
            std::vector<cv::Point2d> inlier_points2d;
            inlier_points3d.reserve(result.inliers.size());
            inlier_points2d.reserve(result.inliers.size());

            for (auto i : result.inliers)
            {
                inlier_points3d.push_back(points3d[i]);
                inlier_points2d.push_back(points2d[i]);
            }

            cv::solvePnPRefineLM(
                inlier_points3d,
                inlier_points2d,
                camera_matrix,
                cv::Mat(),
                result.rvec,
                result.tvec,
                cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 20, 1e-6));
        }

        return result;
    }

    /// Computes reprojection errors for given 3D-2D correspondences
    [[nodiscard]] inline auto compute_reprojection_errors(
        const std::vector<cv::Point3d>& points3d,
        const std::vector<cv::Point2d>& points2d,
        const std::vector<int>& inliers,
        const cv::Mat& rvec,
        const cv::Mat& tvec,
        const cv::Mat& camera_matrix) -> std::vector<double>
    {
        std::vector<cv::Point2d> projected;
        cv::projectPoints(points3d, rvec, tvec, camera_matrix, cv::Mat(), projected);

        std::vector<double> errors;
        errors.reserve(inliers.size());

        for (auto i : inliers)
        {
            errors.push_back(cv::norm(points2d[i] - projected[i]));
        }

        return errors;
    }

    /// Maps indices from correspondence space to feature space
    /// @param correspondence_inliers Inlier indices in correspondence array
    /// @param correspondence_indices Global feature IDs for each correspondence
    template<typename T>
    [[nodiscard]] inline auto map_to_feature_space(
        const std::vector<T>& correspondence_inliers,
        const std::vector<size_t>& correspondence_indices) -> std::vector<size_t>
    {
        std::vector<size_t> feature_inliers;
        feature_inliers.reserve(correspondence_inliers.size());

        for (auto idx : correspondence_inliers)
        {
            if (static_cast<size_t>(idx) < correspondence_indices.size())
                feature_inliers.push_back(correspondence_indices[idx]);
        }

        return feature_inliers;
    }

} // namespace zenslam::pose_estimation

// Include slam_options after other definitions to avoid circular dependencies
#include "zenslam/slam_options.h"
