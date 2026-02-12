#include "zenslam/folder_options.h"

#include <filesystem>

#include <boost/program_options.hpp>

#include <spdlog/spdlog.h>

namespace zenslam
{
    boost::program_options::options_description folder_options::description()
    {
        const folder_options opts;

        boost::program_options::options_description description { "folder options" };

        description.add_options()
        (
            "folder-root",
            boost::program_options::value<std::string>()->default_value(opts.root),
            "Root folder"
        )
        (
            "folder-left",
            boost::program_options::value<std::string>()->default_value(opts.left),
            "Left folder relative to root (or absolute)"
        )
        (
            "folder-right",
            boost::program_options::value<std::string>()->default_value(opts.right),
            "Right folder relative to root (or absolute)"
        )
        (
            "folder-timescale",
            boost::program_options::value<double>()->default_value(opts.timescale),
            "Timescale for folder timestamps"
        )
        (
            "calibration-file",
            boost::program_options::value<std::string>()->default_value(opts.calibration_file),
            "calibration file path"
        )
        (
            "groundtruth-file",
            boost::program_options::value<std::string>()->default_value(opts.groundtruth_file),
            "groundtruth file path"
        )
        (
            "imu-calibration-file",
            boost::program_options::value<std::string>()->default_value(opts.imu_calibration_file),
            "IMU calibration file path"
        )
        (
            "imu-file",
            boost::program_options::value<std::string>()->default_value(opts.imu_file),
            "IMU data CSV file path"
        )
        (
            "folder-output",
            boost::program_options::value<std::string>()->default_value(opts.output),
            "Output folder for results"
        );

        return description;
    }

    void folder_options::validate() const
    {
        if (timescale <= 0.0)
            throw std::invalid_argument("folder.timescale must be > 0");

        // Non-fatal warnings for non-existent paths
        auto resolve = [this](const std::filesystem::path& p) -> std::filesystem::path
        {
            return p.is_absolute() ? p : root / p;
        };

        const auto root_path = root;
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

    void folder_options::print() const
    {
        SPDLOG_INFO("folder root: {}", root.string());
        SPDLOG_INFO("folder left: {}", left.string());
        SPDLOG_INFO("folder right: {}", right.string());
        SPDLOG_INFO("folder output: {}", output.string());
        SPDLOG_INFO("folder timescale: {}", timescale);
        SPDLOG_INFO("calibration file: {}", calibration_file.string());
        SPDLOG_INFO("groundtruth file: {}", groundtruth_file.string());
        SPDLOG_INFO("IMU calibration file: {}", imu_calibration_file.string());
        SPDLOG_INFO("IMU data file: {}", imu_file.string());
    }

} // namespace zenslam