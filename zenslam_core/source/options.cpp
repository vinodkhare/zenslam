#include "zenslam/options.h"

#include <filesystem>

#include <boost/program_options.hpp>

#include <magic_enum/magic_enum.hpp>

#include <spdlog/spdlog.h>

#include <yaml-cpp/yaml.h>

#include "zenslam/folder_options.h"
#include "zenslam/option.h"
#include "zenslam/option_parser.h"
#include "zenslam/option_printer.h"
#include "zenslam/slam_options.h"
#include "zenslam/utils.h"

boost::program_options::options_description zenslam::options::description()
{
    const options options;

    boost::program_options::options_description description { "options" };

    description.add_options()
        (
            "options-file",
            boost::program_options::value<std::string>()->default_value(options.file.value()),
            "options file"
        )
        (
            "log-level",
            boost::program_options::value<std::string>()->default_value(utils::log_levels_to_string[options.log_level]),
            ("log level - pick one of: " + utils::to_string(magic_enum::enum_names<spdlog::level::level_enum>())).c_str()
        )
        ("help,h", "Show help")("version,v", "Show version");

    description.add(folder_options::description());
    description.add(slam_options::description());

    return description;
}

zenslam::options zenslam::options::parse(const int argc, char** argv)
{
    options options { };

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
    for (auto& option : parsed.options)
    {
        options_map[option.string_key] = option;
    }

    if (options_map.contains("options-file"))
        options = parse(map["options-file"].as<std::string>());

    // Parse folder and slam options using their respective parse_cli methods
    folder_options::parse_cli(options.folder, options_map, map);
    slam_options::parse_cli(options.slam, options_map, map);

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
            options.log_level   = option_parser::parse_yaml(options.log_level, application);
            options.log_pattern = option_parser::parse_yaml(options.log_pattern, application);
        }

        if (const auto& folder = config["folder"])
        {
            options.folder = folder_options::parse_yaml(folder);
        }

        if (const auto& slam = config["slam"])
        {
            options.slam = slam_options::parse_yaml(slam);
        }
    }
    catch (const YAML::Exception& e)
    {
        SPDLOG_ERROR("Failed to load config file: {}", e.what());
    }

    return options;
}


void zenslam::options::validate() const
{
    folder.value().validate();
    slam.value().validate();
}

void zenslam::options::print() const
{
    option_printer::print(file);
    option_printer::print(log_level);
    option_printer::print(log_pattern);
    option_printer::print(verb);

    folder.value().print();
    slam.value().print();
}