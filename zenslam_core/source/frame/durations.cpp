#include "zenslam/frame/durations.h"

#include <spdlog/spdlog.h>

namespace zenslam::frame
{
    auto durations::print() const -> void
    {
        SPDLOG_INFO("");
        SPDLOG_INFO("Frame durations:");
        SPDLOG_INFO("  Waiting:       {:+.4f} s", std::chrono::duration<double>(wait).count());
        SPDLOG_INFO("  Preprocessing: {:+.4f} s", std::chrono::duration<double>(processing).count());
        SPDLOG_INFO("  Tracking:      {:+.4f} s", std::chrono::duration<double>(tracking).count());
        SPDLOG_INFO("  Estimation:    {:+.4f} s", std::chrono::duration<double>(estimation).count());
        SPDLOG_INFO("  Total:         {:+.4f} s", std::chrono::duration<double>(total).count());
        SPDLOG_INFO("");
        SPDLOG_INFO("Detailed timing:");
        SPDLOG_INFO("  Detection L:   {:+.4f} s", std::chrono::duration<double>(detection_left).count());
        SPDLOG_INFO("  Detection R:   {:+.4f} s", std::chrono::duration<double>(detection_right).count());
        SPDLOG_INFO("  KLT Tracking:  {:+.4f} s", std::chrono::duration<double>(klt).count());
        SPDLOG_INFO("  Matching:      {:+.4f} s", std::chrono::duration<double>(matching).count());
        SPDLOG_INFO("  Triangulation: {:+.4f} s", std::chrono::duration<double>(triangulation).count());
    }
}