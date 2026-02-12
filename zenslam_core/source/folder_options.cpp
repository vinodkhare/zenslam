#include "zenslam/folder_options.h"

#include <filesystem>

#include <boost/program_options.hpp>

#include <spdlog/spdlog.h>

#include <yaml-cpp/yaml.h>

#include "zenslam/option_parser.h"
#include "zenslam/option_printer.h"

namespace zenslam
{
    auto folder_options::parse_yaml(const YAML::Node& node) -> folder_options
    {
        folder_options options;

        try
        {
            options.root                 = option_parser::parse_yaml(options.root, node);
            options.left                 = option_parser::parse_yaml(options.left, node);
            options.right                = option_parser::parse_yaml(options.right, node);
            options.timescale            = option_parser::parse_yaml(options.timescale, node);
            options.calibration_file     = option_parser::parse_yaml(options.calibration_file, node);
            options.groundtruth_file     = option_parser::parse_yaml(options.groundtruth_file, node);
            options.output               = option_parser::parse_yaml(options.output, node);
            options.imu_calibration_file = option_parser::parse_yaml(options.imu_calibration_file, node);
            options.imu_file             = option_parser::parse_yaml(options.imu_file, node);
        }
        catch (const YAML::Exception& e)
        {
            SPDLOG_ERROR("Error parsing folder options from YAML: {}", e.what());
        }

        return options;
    }

    boost::program_options::options_description folder_options::description()
    {
        const folder_options opts;

        boost::program_options::options_description description { "folder options" };

        description.add_options()("folder-root",
                                  boost::program_options::value<std::string>()->default_value(opts.root.value()),
                                  "Root folder")(
            "folder-left", boost::program_options::value<std::string>()->default_value(opts.left.value()),
            "Left folder relative to root (or absolute)")(
            "folder-right", boost::program_options::value<std::string>()->default_value(opts.right.value()),
            "Right folder relative to root (or absolute)")(
            "folder-timescale", boost::program_options::value<double>()->default_value(opts.timescale),
            "Timescale for folder timestamps")(
            "calibration-file",
            boost::program_options::value<std::string>()->default_value(opts.calibration_file.value()),
            "calibration file path")(
            "groundtruth-file",
            boost::program_options::value<std::string>()->default_value(opts.groundtruth_file.value()),
            "groundtruth file path")(
            "imu-calibration-file",
            boost::program_options::value<std::string>()->default_value(opts.imu_calibration_file.value()),
            "IMU calibration file path")(
            "imu-file", boost::program_options::value<std::string>()->default_value(opts.imu_file.value()),
            "IMU data CSV file path")("folder-output",
                                      boost::program_options::value<std::string>()->default_value(opts.output.value()),
                                      "Output folder for results");

        return description;
    }

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

    void folder_options::print() const
    {
        option_printer::print(root);
        option_printer::print(left);
        option_printer::print(right);
        option_printer::print(output);
        option_printer::print(timescale);
        option_printer::print(calibration_file);
        option_printer::print(groundtruth_file);
        option_printer::print(imu_calibration_file);
        option_printer::print(imu_file);
    }
} // namespace zenslam
