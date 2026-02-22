#pragma once

#include <vector>

#include <opencv2/core/types.hpp>

namespace zenslam::utils
{
    /** Estimate rigid transformation (R, t) between two sets of 3D points.
     *
     * @param points3d_0 First set of 3D points
     * @param points3d_1 Second set of 3D points
     * @param R Output rotation matrix
     * @param t Output translation vector
     * @return True if estimation was successful, false otherwise
     */
    auto estimate_rigid
    (
        const std::vector<cv::Point3d>& points3d_0,
        const std::vector<cv::Point3d>& points3d_1,
        cv::Matx33d&                    R,
        cv::Point3d&                    t
    ) -> bool;

    /** RANSAC wrapper for estimate_rigid: returns best R, t, and inlier/outlier indices
     *
     * @param src Source 3D points
     * @param dst Destination 3D points
     * @param best_R Output best rotation matrix
     * @param best_t Output best translation vector
     * @param inlier_indices Output indices of inliers
     * @param outlier_indices Output indices of outliers
     * @param errors Output errors for inliers
     * @param threshold RANSAC threshold for inlier determination
     * @param max_iterations Maximum RANSAC iterations
     * @return True if successful, false otherwise
     */
    auto estimate_rigid_ransac
    (
        const std::vector<cv::Point3d>& src,
        const std::vector<cv::Point3d>& dst,
        cv::Matx33d&                    best_R,
        cv::Point3d&                    best_t,
        std::vector<size_t>&            inlier_indices,
        std::vector<size_t>&            outlier_indices,
        std::vector<double>&            errors,
        double                          threshold      = 0.01,
        int                             max_iterations = 1000
    ) -> bool;

    /** Compute optimal rigid transformation using Umeyama's method (closed-form solution).
     *
     * @param src Source 3D points
     * @param dst Destination 3D points
     * @param R Output rotation matrix
     * @param t Output translation vector
     */
    auto umeyama
    (
        const std::vector<cv::Point3d>& src,
        const std::vector<cv::Point3d>& dst,
        cv::Matx33d&                    R,
        cv::Point3d&                    t
    ) -> void;
}
