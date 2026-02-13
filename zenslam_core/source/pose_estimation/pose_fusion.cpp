#include "zenslam/pose_estimation/pose_fusion.h"

#include <algorithm>
#include <cmath>
#include <spdlog/spdlog.h>

#include "zenslam/utils_std.h"

namespace zenslam::pose_estimation
{
    auto pose_fusion::compute_weight(const pose_data& pose, const std::string& method_type) -> double
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
        
        // Penalize low inlier ratios heavily
        if (inlier_ratio < 0.3)
            return 0.01 * inlier_ratio;
        
        // Error quality component
        double error_quality = 1.0;
        if (!pose.errors.empty())
        {
            const double mean_error = utils::mean(pose.errors);
            const double std_error = utils::std_dev(pose.errors);
            
            // Check for outlier errors
            const double error_threshold = mean_error + 2.0 * std_error;
            const size_t good_errors = std::ranges::count_if(pose.errors,
                [error_threshold](double err) { return err <= error_threshold; });
            
            // Reduce confidence if too many outlier errors
            const double error_consistency = static_cast<double>(good_errors) / pose.errors.size();
            if (error_consistency < 0.7)
                error_quality *= 0.5;
            
            // Error scale: tuned for typical SLAM reprojection errors
            const double error_scale = (method_type == "3d3d" || method_type == "3d3d_lines") ? 0.1 : 3.0;
            error_quality *= std::exp(-mean_error / error_scale);
        }

        // Feature type reliability
        const double type_weight = method_type.find("lines") != std::string::npos ? 0.9 : 1.0;
        
        // Scale by absolute inlier count
        const double inlier_boost = std::min(1.0, static_cast<double>(inliers) / 50.0);

        // Combine: inlier_ratio (0.4) + error_quality (0.4) + inlier_boost (0.2)
        const double confidence = (inlier_ratio * 0.4 + error_quality * 0.4 + inlier_boost * 0.2) * type_weight;

        return std::clamp(confidence, 0.0, 1.0);
    }

    auto pose_fusion::compute_covariance(
        const pose_data& pose,
        double method_weight,
        const std::string& method_type) -> std::pair<double, double>
    {
        const size_t inliers = pose.inliers.size();
        const size_t total_corr = pose.indices.size();
        
        // No valid pose = maximum uncertainty
        if (inliers < 3 || total_corr == 0 || method_weight < 1e-6)
            return {10.0, 0.5};
        
        // Base uncertainty from inlier ratio
        const double inlier_ratio = static_cast<double>(inliers) / static_cast<double>(total_corr);
        const double ratio_uncertainty = (1.0 - inlier_ratio) / std::sqrt(inlier_ratio);
        
        // Error-based uncertainty
        double error_std = 0.0;
        if (pose.errors.size() > 2)
        {
            double mean_error = utils::mean(pose.errors);
            error_std = utils::std_dev(pose.errors);
            
            // Normalize to meters for 2D methods (pixel to meter conversion)
            if (method_type != "3d3d" && method_type != "3d3d_lines")
            {
                const double pixel_to_meter = 0.01;  // Rough approximation
                mean_error *= pixel_to_meter;
                error_std *= pixel_to_meter;
            }
        }
        
        // Combined translation uncertainty (meters)
        double trans_std = std::sqrt(error_std * error_std + ratio_uncertainty * ratio_uncertainty);
        
        // Rotation uncertainty (radians)
        double rot_std = 0.1 * trans_std;
        
        // Scale by method weight (less confident = more uncertainty)
        trans_std /= (method_weight + 0.1);
        rot_std /= (method_weight + 0.1);
        
        // Clamp to reasonable ranges
        trans_std = std::clamp(trans_std, 0.001, 5.0);
        rot_std = std::clamp(rot_std, 0.0001, 1.0);
        
        return {trans_std, rot_std};
    }

    auto pose_fusion::select_best_rotation(
        const estimate_pose_result& result,
        double w_3d3d, double w_3d2d, double w_2d2d,
        double w_3d3d_lines, double w_3d2d_lines) -> cv::Matx33d
    {
        // Select rotation from method with highest weight
        const double max_weight = std::max({w_3d3d, w_3d2d, w_2d2d, w_3d3d_lines, w_3d2d_lines});

        if (max_weight == w_3d3d && w_3d3d > 0.0)
            return result.pose_3d3d.pose.rotation();
        else if (max_weight == w_3d2d && w_3d2d > 0.0)
            return result.pose_3d2d.pose.rotation();
        else if (max_weight == w_2d2d && w_2d2d > 0.0)
            return result.pose_2d2d.pose.rotation();
        else if (max_weight == w_3d3d_lines && w_3d3d_lines > 0.0)
            return result.pose_3d3d_lines.pose.rotation();
        else if (max_weight == w_3d2d_lines && w_3d2d_lines > 0.0)
            return result.pose_3d2d_lines.pose.rotation();

        return cv::Matx33d::eye();
    }

    auto pose_fusion::fuse_translations(
        const estimate_pose_result& result,
        double w_3d3d, double w_3d2d, double w_2d2d,
        double w_3d3d_lines, double w_3d2d_lines) -> cv::Vec3d
    {
        cv::Vec3d fused = cv::Vec3d::zeros();

        if (w_3d3d > 0)
            fused += result.pose_3d3d.pose.translation() * w_3d3d;
        if (w_3d2d > 0)
            fused += result.pose_3d2d.pose.translation() * w_3d2d;
        if (w_2d2d > 0)
            fused += result.pose_2d2d.pose.translation() * w_2d2d;
        if (w_3d3d_lines > 0)
            fused += result.pose_3d3d_lines.pose.translation() * w_3d3d_lines;
        if (w_3d2d_lines > 0)
            fused += result.pose_3d2d_lines.pose.translation() * w_3d2d_lines;

        return fused;
    }

    auto pose_fusion::fuse_poses(const estimate_pose_result& result) -> weighted_pose_result
    {
        weighted_pose_result fused{};

        // Compute individual weights
        const double w_3d3d = compute_weight(result.pose_3d3d, "3d3d");
        const double w_3d2d = compute_weight(result.pose_3d2d, "3d2d");
        const double w_2d2d = compute_weight(result.pose_2d2d, "2d2d");
        const double w_3d3d_lines = compute_weight(result.pose_3d3d_lines, "3d3d_lines");
        const double w_3d2d_lines = compute_weight(result.pose_3d2d_lines, "3d2d_lines");

        const double weight_sum = w_3d3d + w_3d2d + w_2d2d + w_3d3d_lines + w_3d2d_lines;

        // If no valid poses, return identity
        if (weight_sum < 1e-9)
        {
            fused.pose = cv::Affine3d::Identity();
            fused.confidence = 0.0;
            fused.best_method = "none";
            SPDLOG_DEBUG("Pose fusion: no valid methods, returning identity");
            return fused;
        }

        // Normalize weights to sum to 1.0
        fused.weight_3d3d = w_3d3d / weight_sum;
        fused.weight_3d2d = w_3d2d / weight_sum;
        fused.weight_2d2d = w_2d2d / weight_sum;
        fused.weight_3d3d_lines = w_3d3d_lines / weight_sum;
        fused.weight_3d2d_lines = w_3d2d_lines / weight_sum;

        // Determine best method
        const auto weights = {
            std::make_pair(fused.weight_3d3d, "3D-3D Points"),
            std::make_pair(fused.weight_3d2d, "3D-2D Points"),
            std::make_pair(fused.weight_2d2d, "2D-2D Points"),
            std::make_pair(fused.weight_3d3d_lines, "3D-3D Lines"),
            std::make_pair(fused.weight_3d2d_lines, "3D-2D Lines")
        };
        
        const auto best = std::ranges::max_element(weights, 
            [](const auto& a, const auto& b) { return a.first < b.first; });
        
        fused.best_method = best->second;

        // Set best method inliers
        if (fused.best_method == "3D-3D Points")
            fused.best_method_inliers = result.pose_3d3d.inliers.size();
        else if (fused.best_method == "3D-2D Points")
            fused.best_method_inliers = result.pose_3d2d.inliers.size();
        else if (fused.best_method == "2D-2D Points")
            fused.best_method_inliers = result.pose_2d2d.inliers.size();
        else if (fused.best_method == "3D-3D Lines")
            fused.best_method_inliers = result.pose_3d3d_lines.inliers.size();
        else if (fused.best_method == "3D-2D Lines")
            fused.best_method_inliers = result.pose_3d2d_lines.inliers.size();

        // Fuse pose components
        const cv::Vec3d fused_translation = fuse_translations(
            result, fused.weight_3d3d, fused.weight_3d2d, fused.weight_2d2d,
            fused.weight_3d3d_lines, fused.weight_3d2d_lines);

        const cv::Matx33d fused_rotation = select_best_rotation(
            result, fused.weight_3d3d, fused.weight_3d2d, fused.weight_2d2d,
            fused.weight_3d3d_lines, fused.weight_3d2d_lines);

        fused.pose = cv::Affine3d(fused_rotation, fused_translation);

        // Compute overall confidence as weighted average
        fused.confidence = fused.weight_3d3d * w_3d3d +
                          fused.weight_3d2d * w_3d2d +
                          fused.weight_2d2d * w_2d2d +
                          fused.weight_3d3d_lines * w_3d3d_lines +
                          fused.weight_3d2d_lines * w_3d2d_lines;

        // Total inliers
        fused.total_inliers = result.pose_3d3d.inliers.size() +
                             result.pose_3d2d.inliers.size() +
                             result.pose_2d2d.inliers.size() +
                             result.pose_3d3d_lines.inliers.size() +
                             result.pose_3d2d_lines.inliers.size();

        // Compute covariance from all methods
        const auto [cov_3d3d_t, cov_3d3d_r] = compute_covariance(result.pose_3d3d, fused.weight_3d3d, "3d3d");
        const auto [cov_3d2d_t, cov_3d2d_r] = compute_covariance(result.pose_3d2d, fused.weight_3d2d, "3d2d");
        const auto [cov_2d2d_t, cov_2d2d_r] = compute_covariance(result.pose_2d2d, fused.weight_2d2d, "2d2d");
        const auto [cov_3d3d_lines_t, cov_3d3d_lines_r] = compute_covariance(result.pose_3d3d_lines, fused.weight_3d3d_lines, "3d3d_lines");
        const auto [cov_3d2d_lines_t, cov_3d2d_lines_r] = compute_covariance(result.pose_3d2d_lines, fused.weight_3d2d_lines, "3d2d_lines");

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

        // Build 6x6 covariance matrix (diagonal)
        const double trans_var = fused.translation_std * fused.translation_std;
        const double rot_var = fused.rotation_std * fused.rotation_std;
        
        fused.pose_covariance = cv::Matx66d::zeros();
        fused.pose_covariance(0, 0) = trans_var;
        fused.pose_covariance(1, 1) = trans_var;
        fused.pose_covariance(2, 2) = trans_var;
        fused.pose_covariance(3, 3) = rot_var;
        fused.pose_covariance(4, 4) = rot_var;
        fused.pose_covariance(5, 5) = rot_var;
        
        fused.has_valid_covariance = (fused.total_inliers >= 5);

        // Logging
        SPDLOG_TRACE("Weighted pose fusion:");
        SPDLOG_TRACE("  3D-3D Points: inliers={}, weight={:.3f}", 
            result.pose_3d3d.inliers.size(), fused.weight_3d3d);
        SPDLOG_TRACE("  3D-2D Points: inliers={}, weight={:.3f}",
            result.pose_3d2d.inliers.size(), fused.weight_3d2d);
        SPDLOG_TRACE("  2D-2D Points: inliers={}, weight={:.3f}",
            result.pose_2d2d.inliers.size(), fused.weight_2d2d);
        SPDLOG_TRACE("  3D-3D Lines: inliers={}, weight={:.3f}",
            result.pose_3d3d_lines.inliers.size(), fused.weight_3d3d_lines);
        SPDLOG_TRACE("  3D-2D Lines: inliers={}, weight={:.3f}",
            result.pose_3d2d_lines.inliers.size(), fused.weight_3d2d_lines);
        
        SPDLOG_DEBUG("Pose fusion: best={} (inliers={}), confidence={:.3f}, total_inliers={}, trans_std={:.4f}m, rot_std={:.6f}rad",
            fused.best_method, fused.best_method_inliers, 
            fused.confidence, fused.total_inliers,
            fused.translation_std, fused.rotation_std);

        return fused;
    }

} // namespace zenslam::pose_estimation
