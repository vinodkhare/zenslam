#include "utils.h"

#include <chrono>

std::string zenslam::utils::epoch_double_to_string(const double epoch_seconds)
{
    // Split into integral seconds and fractional milliseconds
    const auto &seconds      = std::chrono::floor<std::chrono::seconds>(std::chrono::duration<double>(epoch_seconds));
    const auto &milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>
            (std::chrono::duration<double>(epoch_seconds) - seconds);

    // sys_time with milliseconds precision
    auto time_point = std::chrono::sys_seconds(seconds) + milliseconds;

    // Format: YYYY-MM-DD HH:MM:SS.mmm UTC
    return std::format("{:%F %T} UTC", time_point);
}
