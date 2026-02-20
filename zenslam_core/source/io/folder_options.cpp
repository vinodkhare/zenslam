#include "zenslam/io/folder_options.h"

#include <filesystem>

#include <spdlog/spdlog.h>

namespace zenslam
{
    // parse_yaml(), parse_cli(), description(), and print() are inherited from options_base

    void folder_options::validate() const
    {
        if (timescale <= 0.0)
            throw std::invalid_argument("folder.timescale must be > 0");

        // Non-fatal warnings for non-existent paths
        auto resolve = [this](const std::filesystem::path& p) -> std::filesystem::path { return p.is_absolute() ? p : root / p; };

        const auto root_path = root.value();
        if (!root_path.empty() && !std::filesystem::exists(root_path))
        {
            SPDLOG_WARN("folder.root does not exist: {}", root_path.string());
        }

        const auto left_path = resolve(left);
        if (!std::filesystem::exists(left_path))
        {
            SPDLOG_WARN("folder.left path does not exist: {}", left_path.string());
        }

        const auto right_path = resolve(right);
        if (!std::filesystem::exists(right_path))
        {
            SPDLOG_WARN("folder.right path does not exist: {}", right_path.string());
        }

        const auto calib_path = resolve(calibration_file);
        if (!std::filesystem::exists(calib_path))
        {
            SPDLOG_WARN("folder.calibration_file not found: {}", calib_path.string());
        }

        const auto gt_path = resolve(groundtruth_file);
        if (!std::filesystem::exists(gt_path))
        {
            SPDLOG_WARN("folder.groundtruth_file not found: {}", gt_path.string());
        }

        const auto imu_path = resolve(imu_calibration_file);
        if (!std::filesystem::exists(imu_path))
        {
            SPDLOG_WARN("folder.imu_calibration_file not found: {}", imu_path.string());
        }
    }

    // print() is now inherited from options_base
} // namespace zenslam
