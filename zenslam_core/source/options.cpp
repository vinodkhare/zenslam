#include "options.h"

#include <print>

#include <boost/program_options.hpp>

#include <magic_enum/magic_enum.hpp>

#include <spdlog/spdlog.h>

#include <yaml-cpp/yaml.h>

#include "utils.h"

boost::program_options::options_description zenslam::options::description()
{
    const options options;

    boost::program_options::options_description description { "options" };

    description.add_options()
    (
        "options-file",
        boost::program_options::value<std::string>()->default_value(options.file),
        "options file"
    )
    (
        "log-level",
        boost::program_options::value<std::string>()->default_value(utils::log_levels_to_string[options.log_level]),
        ("log level - pick one of: " + utils::to_string(magic_enum::enum_names<spdlog::level::level_enum>())).c_str()
    )
    (
        "clahe-enabled",
        boost::program_options::value<bool>()->default_value(options.slam.clahe_enabled),
        "CLAHE enabled"
    )
    (
        "epipolar-threshold",
        boost::program_options::value<double>()->default_value(options.slam.epipolar_threshold),
        "Epipolar threshold"
    )
    (
        "feature",
        boost::program_options::value<std::string>()->default_value(std::string(magic_enum::enum_name(options.slam.feature))),
        ("feature type - pick one of: " + utils::to_string(magic_enum::enum_names<feature_type>())).c_str()
    )
    (
        "descriptor",
        boost::program_options::value<std::string>()->default_value
        (std::string(magic_enum::enum_name(options.slam.descriptor))),
        ("descriptor type - pick one of: " + utils::to_string(magic_enum::enum_names<descriptor_type>())).c_str()
    )
    (
        "fast-threshold",
        boost::program_options::value<double>()->default_value(options.slam.fast_threshold),
        "FAST threshold"
    )
    (
        "threshold-3d3d",
        boost::program_options::value<double>()->default_value(options.slam.threshold_3d3d),
        "3D-3D RANSAC pose estimation threshold in meters"
    )
    (
        "threshold-3d2d",
        boost::program_options::value<double>()->default_value(options.slam.threshold_3d2d),
        "3D-2D RANSAC pose estimation threshold in pixels"
    )("help,h", "Show help")("version,v", "Show version");

    description.add(folder::description());

    return description;
}

zenslam::options zenslam::options::parse(const int argc, char** argv)
{
    options          options { };
    zenslam::options options_default { };

    const auto& description = options::description(); // inlining this can cause argument parse failures, don't know why!
    const auto& parsed      = parse_command_line(argc, argv, description);
    auto        map         = boost::program_options::variables_map();

    store(parsed, map);
    notify(map);

    if (parsed.options.empty() || map.contains("help"))
    {
        options.verb = verb::HELP;
    }

    if (map.contains("version"))
    {
        options.verb = verb::VERSION;
    }

    std::map<std::string, boost::program_options::basic_option<char>> options_map { };
    for (auto& option: parsed.options)
    {
        options_map[option.string_key] = option;
    }

    if (options_map.contains("options-file")) options = parse(map["options-file"].as<std::string>());

    if (options_map.contains("folder-root")) options.folder.root = map["folder-root"].as<std::string>();
    if (options_map.contains("folder-left")) options.folder.left = map["folder-left"].as<std::string>();
    if (options_map.contains("folder-right")) options.folder.right = map["folder-right"].as<std::string>();
    if (options_map.contains("folder-output")) options.folder.output = map["folder-output"].as<std::string>();
    if (options_map.contains("folder-timescale")) options.folder.timescale = map["folder-timescale"].as<double>();
    if (options_map.contains("calibration-file")) options.folder.calibration_file = map["calibration-file"].as<std::string>();
    if (options_map.contains("grouthtruth-file")) options.folder.groundtruth_file = map["grouthtruth-file"].as<std::string>();
    if (options_map.contains
        ("imu-calibration-file"))
        options.folder.imu_calibration_file = map["imu-calibration-file"].as<std::string>();

    if (options_map.contains("options-file")) options.file = map["options-file"].as<std::string>();
    if (options_map.contains
        ("log-level"))
        options.log_level = utils::log_levels_from_string[map["log-level"].as<std::string>()];
    if (options_map.contains("log-pattern")) options.log_pattern = map["log-pattern"].as<std::string>();
    if (options_map.contains("clahe-enabled")) options.slam.clahe_enabled = map["clahe-enabled"].as<bool>();
    if (options_map.contains("epipolar-threshold")) options.slam.epipolar_threshold = map["epipolar-threshold"].as<double>();
    if (options_map.contains("feature"))
        options.slam.feature = magic_enum::enum_cast<feature_type>
                (map["feature"].as<std::string>()).value_or(options.slam.feature);
    if (options_map.contains("descriptor"))
        options.slam.descriptor = magic_enum::enum_cast<descriptor_type>
                (map["descriptor"].as<std::string>()).value_or(options.slam.descriptor);
    if (options_map.contains("fast-threshold")) options.slam.fast_threshold = map["fast-threshold"].as<int>();
    if (options_map.contains("threshold-3d3d")) options.slam.threshold_3d3d = map["threshold-3d3d"].as<double>();
    if (options_map.contains("threshold-3d2d")) options.slam.threshold_3d2d = map["threshold-3d2d"].as<double>();

    return options;
}

zenslam::options zenslam::options::parse(const std::filesystem::path& path)
{
    options options { };

    options.file = path;

    try
    {
        auto config = YAML::LoadFile(path.string());

        if (const auto& application = config["application"])
        {
            options.log_level   = utils::log_levels_from_string[application["log_level"].as<std::string>()];
            options.log_pattern = application["log_pattern"].as<std::string>();
        }

        if (const auto& folder = config["folder"])
        {
            options.folder.root             = folder["root"].as<std::string>();
            options.folder.left             = folder["left"].as<std::string>();
            options.folder.right            = folder["right"].as<std::string>();
            options.folder.timescale        = folder["timescale"].as<double>();
            options.folder.calibration_file = folder["calibration_file"].as<std::string>();
            options.folder.groundtruth_file = folder["groundtruth_file"].as<std::string>();

            if (folder["output"])
            {
                options.folder.output = folder["output"].as<std::string>();
            }

            if (folder["imu_calibration_file"])
            {
                options.folder.imu_calibration_file = folder["imu_calibration_file"].as<std::string>();
            }
        }

        if (const auto& slam = config["slam"])
        {
            if (const auto cell_size = slam["cell_size"])
            {
                const auto cell_array         = cell_size.as<std::vector<int>>();
                options.slam.cell_size.width  = cell_array[0];
                options.slam.cell_size.height = cell_array[1];
            }

            if (const auto clahe_enabled = slam["clahe_enabled"])
            {
                options.slam.clahe_enabled = clahe_enabled.as<bool>();
            }

            if (const auto epipolar_threshold = slam["threshold_epipolar"])
            {
                options.slam.epipolar_threshold = epipolar_threshold.as<double>();
            }

            if (const auto feature = slam["feature"])
            {
                options.slam.feature =
                        magic_enum::enum_cast<feature_type>(feature.as<std::string>()).value_or(options.slam.feature);
            }

            if (const auto descriptor = slam["descriptor"])
            {
                options.slam.descriptor = magic_enum::enum_cast<descriptor_type>(descriptor.as<std::string>())
                       .value_or(options.slam.descriptor);
            }

            if (const auto fast_threshold = slam["fast_threshold"])
            {
                options.slam.fast_threshold = fast_threshold.as<int>();
            }

            if (const auto klt_window_size = slam["klt_window_size"])
            {
                const auto klt_window_array         = klt_window_size.as<std::vector<int>>();
                options.slam.klt_window_size.width  = klt_window_array[0];
                options.slam.klt_window_size.height = klt_window_array[1];
            }

            if (const auto klt_max_level = slam["klt_max_level"])
            {
                options.slam.klt_max_level = klt_max_level.as<int>();
            }

            if (const auto threshold_3d3d = slam["threshold_3d3d"])
            {
                options.slam.threshold_3d3d = threshold_3d3d.as<double>();
            }

            if (const auto threshold_3d2d = slam["threshold_3d2d"])
            {
                options.slam.threshold_3d2d = threshold_3d2d.as<double>();
            }
        }
    }
    catch (const YAML::Exception& e)
    {
        std::println("Failed to load config file: {}", e.what());
    }

    return options;
}

boost::program_options::options_description zenslam::options::folder::description()
{
    const folder folder;

    boost::program_options::options_description description { "folder options" };

    description.add_options()
    (
        "folder-root",
        boost::program_options::value<std::string>()->default_value(folder.root),
        "Root folder"
    )
    (
        "folder-left",
        boost::program_options::value<std::string>()->default_value(folder.left),
        "Left folder relative to root (or absolute)"
    )
    (
        "folder-right",
        boost::program_options::value<std::string>()->default_value(folder.right),
        "Right folder relative to root (or absolute)"
    )
    (
        "folder-timescale",
        boost::program_options::value<double>()->default_value(folder.timescale),
        "Timescale for folder timestamps"
    )
    (
        "calibration-file",
        boost::program_options::value<std::string>()->default_value(folder.calibration_file),
        "calibration file path"
    )
    (
        "grouthtruth-file",
        boost::program_options::value<std::string>()->default_value(folder.groundtruth_file),
        "groundtruth file path"
    )
    (
        "imu-calibration-file",
        boost::program_options::value<std::string>()->default_value(folder.imu_calibration_file),
        "IMU calibration file path"
    )
    (
        "folder-output",
        boost::program_options::value<std::string>()->default_value(folder.output),
        "Output folder for results"
    );

    return description;
}

void zenslam::options::folder::print() const
{
    SPDLOG_INFO("folder root: {}", root.string());
    SPDLOG_INFO("folder left: {}", left.string());
    SPDLOG_INFO("folder right: {}", right.string());
    SPDLOG_INFO("folder output: {}", output.string());
    SPDLOG_INFO("folder timescale: {}", timescale);
    SPDLOG_INFO("calibration file: {}", calibration_file.string());
    SPDLOG_INFO("groundtruth file: {}", groundtruth_file.string());
    SPDLOG_INFO("IMU calibration file: {}", imu_calibration_file.string());
}

void zenslam::options::slam::print() const
{
    SPDLOG_INFO("cell size: [{}, {}]", cell_size.width, cell_size.height);
    SPDLOG_INFO("CLAHE enabled: {}", clahe_enabled ? "true" : "false");
    SPDLOG_INFO("epipolar threshold: {}", epipolar_threshold);
    SPDLOG_INFO("feature type: {}", magic_enum::enum_name(feature));
    SPDLOG_INFO("descriptor type: {}", magic_enum::enum_name(descriptor));
    SPDLOG_INFO("fast threshold: {}", fast_threshold);
    SPDLOG_INFO("klt window size: [{}, {}]", klt_window_size.width, klt_window_size.height);
    SPDLOG_INFO("klt max level: {}", klt_max_level);
    SPDLOG_INFO("3D-3D RANSAC threshold: {} m", threshold_3d3d);
    SPDLOG_INFO("3D-2D RANSAC threshold: {} px", threshold_3d2d);
}

void zenslam::options::print() const
{
    SPDLOG_INFO("file: {}", file.string());
    SPDLOG_INFO("log level: {}", magic_enum::enum_name(log_level));
    SPDLOG_INFO("log pattern: {}", log_pattern);

    folder.print();
    slam.print();
}