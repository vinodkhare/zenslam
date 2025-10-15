#include "utils_std.h"

#include <chrono>
#include <format>

#include <gsl/narrow>

auto zenslam::utils::mean(const std::vector<double> &values) -> double
{
    auto mean = 0.0;
    for (const auto &value: values)
    {
        mean += value;
    }
    return mean / gsl::narrow<double>(values.size());
}

auto zenslam::utils::to_string(const std::vector<std::string> &strings, const std::string &delimiter) -> std::string
{
    return join_to_string(strings, delimiter, std::identity { });
}



auto zenslam::utils::to_string(const std::vector<double> &values, const std::string &delimiter) -> std::string
{
    return join_to_string
    (
        values,
        delimiter,
        [](const double value)
        {
            return std::to_string(value);
        }
    );
}

std::string zenslam::utils::to_string_epoch(const double epoch_seconds)
{
    // Split into integral seconds and fractional milliseconds
    const auto &seconds      = std::chrono::floor<std::chrono::seconds>(std::chrono::duration<double>(epoch_seconds));
    const auto &milliseconds =
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(epoch_seconds) - seconds);

    // sys_time with milliseconds precision
    auto time_point = std::chrono::sys_seconds(seconds) + milliseconds;

    // Format: YYYY-MM-DD HH:MM:SS.mmm UTC
    return std::format("{:%F %T} UTC", time_point);
}
