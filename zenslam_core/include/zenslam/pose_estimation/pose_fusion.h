#pragma once

#include <string>
#include <opencv2/core.hpp>

#include "zenslam/utils/pose_data.h"

namespace zenslam::pose_estimation
{
    /// Result from weighted pose fusion combining multiple estimation methods
    struct weighted_pose_result
    {
        cv::Affine3d pose;                    // Fused pose estimate
        double       confidence { 0.0 };      // Overall confidence score (0-1)
        int          total_inliers { 0 };     // Total inliers across all methods
        
        // Individual method contributions
        double       weight_3d3d { 0.0 };
        double       weight_3d2d { 0.0 };
        double       weight_2d2d { 0.0 };
        double       weight_3d3d_lines { 0.0 };
        double       weight_3d2d_lines { 0.0 };
        
        // Best contributing method
        std::string  best_method;
        size_t       best_method_inliers { 0 };
        
        // Uncertainty quantification
        cv::Matx66d  pose_covariance { };              // 6x6 uncertainty matrix (SE(3))
        double       translation_std { 0.0 };          // Translation std dev (meters)
        double       rotation_std { 0.0 };             // Rotation std dev (radians)
        bool         has_valid_covariance { false };   // Whether covariance is valid
    };

    /// Result structure from multi-method pose estimation
    struct estimate_pose_result
    {
        pose_data    pose_3d3d;       // 3D-3D point result (maybe empty)
        pose_data    pose_3d2d;       // 3D-2D point result (maybe empty)
        pose_data    pose_2d2d;       // 2D-2D point result scaled using previous triangulated points (maybe empty)
        pose_data    pose_3d3d_lines; // 3D-3D line result (maybe empty)
        pose_data    pose_3d2d_lines; // 3D-2D line result (maybe empty)
        cv::Affine3d chosen_pose;     // Selected pose (or identity)
        size_t       chosen_count { };
    };

    /// Confidence-based fusion of multiple pose estimates
    /// Uses modern C++23 features for cleaner implementation
    class pose_fusion
    {
    public:
        /// Compute weighted fusion of multiple pose estimation methods
        ///
        /// Combines all available pose estimates using confidence weights based on:
        /// - Inlier ratio (inliers vs total correspondences)
        /// - Mean reprojection error (lower error = higher confidence)
        /// - Feature type reliability (points > lines)
        [[nodiscard]] static auto fuse_poses(const estimate_pose_result& result) -> weighted_pose_result;

    private:
        /// Compute confidence weight for a single pose estimate
        /// @param pose_data The pose estimate with inliers/outliers/errors
        /// @param method_type "3d3d", "3d2d", "2d2d", etc.
        /// @return Confidence weight in range [0, 1]
        [[nodiscard]] static auto compute_weight(const pose_data& pose, const std::string& method_type) -> double;

        /// Compute covariance uncertainty for a single pose estimate
        /// @return Pair of (translation_std, rotation_std)
        [[nodiscard]] static auto compute_covariance(
            const pose_data& pose,
            double method_weight,
            const std::string& method_type) -> std::pair<double, double>;

        /// Select rotation from best method (rotation averaging is problematic)
        [[nodiscard]] static auto select_best_rotation(const estimate_pose_result& result, 
            double w_3d3d, double w_3d2d, double w_2d2d, 
            double w_3d3d_lines, double w_3d2d_lines) -> cv::Matx33d;

        /// Fuse translations using weighted average
        [[nodiscard]] static auto fuse_translations(const estimate_pose_result& result,
            double w_3d3d, double w_3d2d, double w_2d2d,
            double w_3d3d_lines, double w_3d2d_lines) -> cv::Vec3d;
    };

} // namespace zenslam::pose_estimation
