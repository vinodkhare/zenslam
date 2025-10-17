#include "frame_writer.h"

zenslam::frame_writer::frame_writer(const std::filesystem::path &path) :
    _file { path }
{
    _file << "timestamp, t_preprocessing, t_tracking, t_detection, t_matching, t_estimation, t_total\n";
}

auto zenslam::frame_writer::write(slam_frame &frame) -> void

{
    _file << std::format
    (
        "{:+.3f}, {:+.3f}, {:+.3f}, {:+.3f}, {:+.3f}, {:+.3f}, {:+.3f}\n",
        frame.frame[1].l.timestamp,
        std::chrono::duration<double>(frame.durations.preprocessing).count(),
        std::chrono::duration<double>(frame.durations.tracking).count(),
        std::chrono::duration<double>(frame.durations.detection).count(),
        std::chrono::duration<double>(frame.durations.matching).count(),
        std::chrono::duration<double>(frame.durations.estimation).count(),
        std::chrono::duration<double>(frame.durations.total).count()
    );
}
