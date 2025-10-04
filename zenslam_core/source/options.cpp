#include "options.h"

#include <print>
#include <yaml-cpp/yaml.h>
#include <boost/program_options.hpp>

#include "utils.h"
#include <magic_enum/magic_enum.hpp>

#include <spdlog/spdlog.h>

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
        "epipolar-threshold",
        boost::program_options::value<double>()->default_value(options.slam.epipolar_threshold),
        "Epipolar threshold"
    )
    (
        "fast-threshold",
        boost::program_options::value<double>()->default_value(options.slam.fast_threshold),
        "FAST threshold"
    )
    (
        "help,h",
        "Show help"
    )
    (
        "version,v",
        "Show version"
    );

    description.add(folder::description());

    return description;
}

zenslam::options zenslam::options::parse(const int argc, char **argv)
{
    options          options { };
    zenslam::options options_default { };

    const auto &description = options::description(); // inlining this can cause argument parse failures, don't know why!
    const auto &parsed      = parse_command_line(argc, argv, description);
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

    const auto options_map = utils::to_map(parsed.options);

    if (options_map.contains("options-file")) options = parse(map["options-file"].as<std::string>());

    if (options_map.contains("folder-root")) options.folder.root = map["folder-root"].as<std::string>();
    if (options_map.contains("folder-left")) options.folder.left = map["folder-left"].as<std::string>();
    if (options_map.contains("folder-right")) options.folder.right = map["folder-right"].as<std::string>();
    if (options_map.contains("folder-timescale")) options.folder.timescale = map["folder-timescale"].as<double>();
    if (options_map.contains("calibration-file")) options.folder.calibration_file = map["calibration-file"].as<std::string>();
    if (options_map.contains("options-file")) options.file = map["options-file"].as<std::string>();
    if (options_map.contains
        ("log-level"))
        options.log_level = utils::log_levels_from_string[map["log-level"].as<std::string>()];
    if (options_map.contains("log-pattern")) options.log_pattern = map["log-pattern"].as<std::string>();
    if (options_map.contains("epipolar-threshold")) options.slam.epipolar_threshold = map["epipolar-threshold"].as<double>();
    if (options_map.contains("fast-threshold")) options.slam.fast_threshold = map["fast-threshold"].as<double>();

    return options;
}

zenslam::options zenslam::options::parse(const std::filesystem::path &path)
{
    options options { };

    options.file = path;

    try
    {
        auto config = YAML::LoadFile(path.string());

        if (const auto &application = config["application"])
        {
            options.log_level   = utils::log_levels_from_string[application["log_level"].as<std::string>()];
            options.log_pattern = application["log_pattern"].as<std::string>();
        }

        if (const auto &folder = config["folder"])
        {
            options.folder.root             = folder["root"].as<std::string>();
            options.folder.left             = folder["left"].as<std::string>();
            options.folder.right            = folder["right"].as<std::string>();
            options.folder.timescale        = folder["timescale"].as<double>();
            options.folder.calibration_file = folder["calibration_file"].as<std::string>();
        }

        if (const auto &slam = config["slam"])
        {
            if (const auto cell_size = slam["cell_size"])
            {
                const auto cell_array         = cell_size.as<std::vector<int>>();
                options.slam.cell_size.width  = cell_array[0];
                options.slam.cell_size.height = cell_array[1];
            }

            if (const auto epipolar_threshold = slam["epipolar_threshold"])
            {
                options.slam.epipolar_threshold = epipolar_threshold.as<double>();
            }

            if (const auto fast_threshold = slam["fast_threshold"])
            {
                options.slam.fast_threshold = fast_threshold.as<int>();
            }
        }
    }
    catch (const YAML::Exception &e)
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
    );

    return description;
}

void zenslam::options::folder::print() const
{
    SPDLOG_INFO("folder root: {}", root.string());
    SPDLOG_INFO("folder left: {}", left.string());
    SPDLOG_INFO("folder right: {}", right.string());
    SPDLOG_INFO("folder timescale: {}", timescale);
    SPDLOG_INFO("calibration file: {}", calibration_file.string());
}

void zenslam::options::slam::print() const
{
    SPDLOG_INFO("cell size: [{}, {}]", cell_size.width, cell_size.height);
    SPDLOG_INFO("epipolar threshold: {}", epipolar_threshold);
    SPDLOG_INFO("fast threshold: {}", fast_threshold);
}

void zenslam::options::print() const
{
    SPDLOG_INFO("file: {}", file.string());
    SPDLOG_INFO("log level: {}", magic_enum::enum_name(log_level));
    SPDLOG_INFO("log pattern: {}", log_pattern);

    folder.print();
    slam.print();
}
