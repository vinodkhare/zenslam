#include "frame/durations.h"
#include <spdlog/spdlog.h>

namespace zenslam::frame
{
    auto durations::print() const -> void
    {
        SPDLOG_INFO("");
        SPDLOG_INFO("Frame durations:");
        SPDLOG_INFO("  Preprocessing: {:+.4f} s", std::chrono::duration<double>(preprocessing).count());
        SPDLOG_INFO("  Tracking:      {:+.4f} s", std::chrono::duration<double>(tracking).count());
        SPDLOG_INFO("  Detection:     {:+.4f} s", std::chrono::duration<double>(detection).count());
        SPDLOG_INFO("  Matching:      {:+.4f} s", std::chrono::duration<double>(matching).count());
        SPDLOG_INFO("  Estimation:    {:+.4f} s", std::chrono::duration<double>(estimation).count());
        SPDLOG_INFO("  Total:         {:+.4f} s", std::chrono::duration<double>(total).count());
    }
}
