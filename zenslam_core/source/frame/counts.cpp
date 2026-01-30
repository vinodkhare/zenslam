#include "zenslam/frame/counts.h"

#include <spdlog/spdlog.h>

namespace zenslam::frame
{
    auto counts::print() const -> void
    {
        SPDLOG_INFO("");
        SPDLOG_INFO("Counts:");
        SPDLOG_INFO("  Keypoints L:           {:4}", keypoints_l);
        SPDLOG_INFO("  Keypoints R:           {:4}", keypoints_r);
        SPDLOG_INFO("  Keypoints L Tracked:   {:4}", keypoints_l_tracked);
        SPDLOG_INFO("  Keypoints R Tracked:   {:4}", keypoints_r_tracked);
        SPDLOG_INFO("  Keypoints L New:       {:4}", keypoints_l_new);
        SPDLOG_INFO("  Keypoints R New:       {:4}", keypoints_r_new);
        SPDLOG_INFO("  Keypoints Total:       {:4}", keypoints_total);
        SPDLOG_INFO("  Matches:               {:4}", matches);
        SPDLOG_INFO("  Matches Triangulated:  {:4}", matches_triangulated);
        SPDLOG_INFO("  3D-3D Correspondences: {:4}", correspondences_3d3d);
        SPDLOG_INFO("  3D-2D Correspondences: {:4}", correspondences_3d2d);
        SPDLOG_INFO("  2D-2D Correspondences: {:4}", correspondences_2d2d);
        SPDLOG_INFO("  3D-3D Inliers:         {:4}", correspondences_3d3d_inliers);
        SPDLOG_INFO("  3D-2D Inliers:         {:4}", correspondences_3d2d_inliers);
        SPDLOG_INFO("  2D-2D Inliers:         {:4}", correspondences_2d2d_inliers);
        SPDLOG_INFO("  Map Points:            {:4}", points);
        SPDLOG_INFO("");
        SPDLOG_INFO("Quality Metrics:");
        SPDLOG_INFO("  KLT Error Mean:        {:.3f} px", klt_error_mean);
        SPDLOG_INFO("  KLT Success Rate:      {:.1f}%", klt_success_rate * 100.0);
        SPDLOG_INFO("  Match Distance Mean:   {:.3f}", match_distance_mean);
        SPDLOG_INFO("  Epipolar Error Mean:   {:.3f} px", epipolar_error_mean);
        SPDLOG_INFO("  Fundamental Inliers:   {:4}", fundamental_inliers);
        SPDLOG_INFO("  Response Mean L/R:     {:.1f} / {:.1f}", response_mean_l, response_mean_r);
        SPDLOG_INFO("  Grid Occupancy L/R:    {:.1f}% / {:.1f}%", grid_occupancy_l * 100.0, grid_occupancy_r * 100.0);
        SPDLOG_INFO("  Track Age Mean/Max:    {:.1f} / {:.0f}", track_age_mean, track_age_max);
    }
}