#include "frame_counts.h"

#include <spdlog/spdlog.h>

auto zenslam::frame_counts::print() const -> void
{
    SPDLOG_INFO("");
    SPDLOG_INFO("Counts:");
    SPDLOG_INFO("  Keypoints L:           {:4}", keypoints_l);
    SPDLOG_INFO("  Keypoints R:           {:4}", keypoints_r);
    SPDLOG_INFO("  Keypoints L Tracked:   {:4}", keypoints_l_tracked);
    SPDLOG_INFO("  Keypoints R Tracked:   {:4}", keypoints_r_tracked);
    SPDLOG_INFO("  Matches:               {:4}", matches);
    SPDLOG_INFO("  Matches Triangulated:  {:4}", maches_triangulated);
    SPDLOG_INFO("  3D-3D Correspondences: {:4}", correspondences_3d3d);
    SPDLOG_INFO("  3D-2D Correspondences: {:4}", correspondences_3d2d);
    SPDLOG_INFO("  3D-3D Inliers:         {:4}", correspondences_3d3d_inliers);
    SPDLOG_INFO("  3D-2D Inliers:         {:4}", correspondences_3d2d_inliers);
}
