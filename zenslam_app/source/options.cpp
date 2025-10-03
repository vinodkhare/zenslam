#include "options.h"

#include <print>
#include <yaml-cpp/yaml.h>
#include <boost/program_options.hpp>

zenslam::options zenslam::options::read(const std::filesystem::path &path)
{
    options options { };

    options.file = path;

    try
    {
        auto config = YAML::LoadFile(path.string());

        if (const auto &folder = config["folder"])
        {
            options.folder.root      = folder["root"].as<std::string>();
            options.folder.left      = folder["left"].as<std::string>();
            options.folder.right     = folder["right"].as<std::string>();
            options.folder.timescale = folder["timescale"].as<double>();
        }

        if (const auto &slam = config["slam"])
        {
            if (const auto cell_size = slam["cell_size"])
            {
                const auto cell_array         = cell_size.as<std::vector<int> >();
                options.slam.cell_size.width  = cell_array[0];
                options.slam.cell_size.height = cell_array[1];
            }
        }
    }
    catch (const YAML::Exception &e)
    {
        std::println("Failed to load config file: {}", e.what());
    }

    return options;
}

zenslam::options zenslam::options::read(const boost::program_options::variables_map &map)
{
    options options { };

    if (map.contains
        ("folder-root") && options.folder.root != map["folder-root"].as<std::string>())
        options.folder.root = map["folder-root"].as<std::string>();
    if (map.contains
        ("folder-left") && options.folder.left != map["folder-left"].as<std::string>())
        options.folder.left = map["folder-left"].as<std::string>();
    if (map.contains
        ("folder-right") && options.folder.right != map["folder-right"].as<std::string>())
        options.folder.right = map["folder-right"].as<std::string>();
    if (map.contains
        ("folder-timescale") && options.folder.timescale != map["folder-timescale"].as<double>())
        options.folder.timescale = map["folder-timescale"].as<double>();
    if (map.contains
        ("options-file") && options.file != map["options-file"].as<std::string>())
        options.file = map["options-file"].as<std::string>();

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
    );

    return description;
}

void zenslam::options::folder::print() const
{
    std::println("folder root: {}", root.string());
    std::println("folder left: {}", left.string());
    std::println("folder right: {}", right.string());
    std::println("folder timescale: {}", timescale);
}

void zenslam::options::slam::print() const
{
    std::println("cell size: [{}, {}]", cell_size.width, cell_size.height);
}

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

void zenslam::options::print() const
{
    std::println("file: {}", file.string());

    folder.print();
    slam.print();
}
