#include "frame_writer.h"

zenslam::frame_writer::frame_writer(const std::filesystem::path &path)
{
    auto path_canonical = std::filesystem::absolute(path);

    if (!std::filesystem::exists(path.parent_path()))
    {
        std::filesystem::create_directories(path.parent_path());
    }

    _file = std::ofstream(path_canonical, std::ios::out | std::ios::trunc);

    _file <<
            "timestamp, t_preprocessing, t_tracking, t_detection, t_matching, t_estimation, t_total, n_keypoints_l, n_keypoints_r, n_tracked_l, n_tracked_r, n_matches, n_triangulated, n_3d3d, n_3d2d, n_3d3d_inliers, n_3d2d_inliers\n";
}

auto zenslam::frame_writer::write(slam_frame &frame) -> void
{
    _file
            << std::format
            (
                "{:+.3f}, {:+.3f}, {:+.3f}, {:+.3f}, {:+.3f}, {:+.3f}, {:+.3f}",
                frame.frames[1].cameras[0].timestamp,
                std::chrono::duration<double>(frame.durations.preprocessing).count(),
                std::chrono::duration<double>(frame.durations.tracking).count(),
                std::chrono::duration<double>(frame.durations.detection).count(),
                std::chrono::duration<double>(frame.durations.matching).count(),
                std::chrono::duration<double>(frame.durations.estimation).count(),
                std::chrono::duration<double>(frame.durations.total).count()
            )
            << std::format
            (
                ", {:4}, {:4}, {:4}, {:4}, {:4}, {:4}, {:4}, {:4}, {:4}, {:4}\n",
                frame.counts.keypoints_l,
                frame.counts.keypoints_r,
                frame.counts.keypoints_l_tracked,
                frame.counts.keypoints_r_tracked,
                frame.counts.matches,
                frame.counts.maches_triangulated,
                frame.counts.correspondences_3d3d,
                frame.counts.correspondences_3d2d,
                frame.counts.correspondences_3d3d_inliers,
                frame.counts.correspondences_3d2d_inliers
            );
}
