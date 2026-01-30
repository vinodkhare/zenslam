#include "zenslam/frame/writer.h"

#include <filesystem>
#include <format>

namespace zenslam::frame
{
    writer::writer(const std::filesystem::path& path)
    {
        auto path_canonical = std::filesystem::absolute(path);
        if (!std::filesystem::exists(path.parent_path()))
        {
            std::filesystem::create_directories(path.parent_path());
        }
        _file = std::ofstream(path_canonical, std::ios::out | std::ios::trunc);
        _file <<
            "timestamp,"
            "t_wait,t_preprocessing,t_tracking,t_estimation,t_total,"
            "t_detection_left,t_detection_right,t_klt,t_matching,t_triangulation,"
            "n_keypoints_l,n_keypoints_r,n_tracked_l,n_tracked_r,n_new_l,n_new_r,"
            "n_matches,n_triangulated,"
            "n_3d3d,n_3d2d,n_2d2d,n_3d3d_inliers,n_3d2d_inliers,n_2d2d_inliers,"
            "klt_error_mean,klt_error_std,klt_success_rate,"
            "match_distance_mean,match_distance_std,"
            "epipolar_error_mean,fundamental_inliers,"
            "response_mean_l,response_mean_r,response_std_l,response_std_r,"
            "grid_occupancy_l,grid_occupancy_r,"
            "track_age_mean,track_age_max,"
            "detector_type,descriptor_type,matcher_type\n";
    }

    auto writer::write(system& frame) -> void
    {
        _file << std::format
            (
                "{:.3f},"
                "{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},"
                "{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},"
                "{},{},{},{},{},{},"
                "{},{},"
                "{},{},{},{},{},{},"
                "{:.3f},{:.3f},{:.4f},"
                "{:.3f},{:.3f},"
                "{:.3f},{},"
                "{:.2f},{:.2f},{:.2f},{:.2f},"
                "{:.4f},{:.4f},"
                "{:.2f},{:.0f},"
                "{},{},{}\n",
                frame[1].timestamp,
                // Durations
                std::chrono::duration<double>(frame.durations.wait).count(),
                std::chrono::duration<double>(frame.durations.processing).count(),
                std::chrono::duration<double>(frame.durations.tracking).count(),
                std::chrono::duration<double>(frame.durations.estimation).count(),
                std::chrono::duration<double>(frame.durations.total).count(),
                std::chrono::duration<double>(frame.durations.detection_left).count(),
                std::chrono::duration<double>(frame.durations.detection_right).count(),
                std::chrono::duration<double>(frame.durations.klt).count(),
                std::chrono::duration<double>(frame.durations.matching).count(),
                std::chrono::duration<double>(frame.durations.triangulation).count(),
                // Keypoint counts
                frame.counts.keypoints_l,
                frame.counts.keypoints_r,
                frame.counts.keypoints_l_tracked,
                frame.counts.keypoints_r_tracked,
                frame.counts.keypoints_l_new,
                frame.counts.keypoints_r_new,
                frame.counts.matches,
                frame.counts.matches_triangulated,
                // Correspondence counts
                frame.counts.correspondences_3d3d,
                frame.counts.correspondences_3d2d,
                frame.counts.correspondences_2d2d,
                frame.counts.correspondences_3d3d_inliers,
                frame.counts.correspondences_3d2d_inliers,
                frame.counts.correspondences_2d2d_inliers,
                // Quality metrics
                frame.counts.klt_error_mean,
                frame.counts.klt_error_std,
                frame.counts.klt_success_rate,
                frame.counts.match_distance_mean,
                frame.counts.match_distance_std,
                frame.counts.epipolar_error_mean,
                frame.counts.fundamental_inliers,
                frame.counts.response_mean_l,
                frame.counts.response_mean_r,
                frame.counts.response_std_l,
                frame.counts.response_std_r,
                frame.counts.grid_occupancy_l,
                frame.counts.grid_occupancy_r,
                frame.counts.track_age_mean,
                frame.counts.track_age_max,
                // Configuration
                frame.counts.detector_type,
                frame.counts.descriptor_type,
                frame.counts.matcher_type
            );
    }
}