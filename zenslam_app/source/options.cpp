#include "options.h"

#include <print>
#include <yaml-cpp/yaml.h>

zenslam::options zenslam::options::read(const std::filesystem::path &path)
{
    options opts;

    try
    {
        auto config = YAML::LoadFile(path.string());

        if (const auto folder = config["folder"])
        {
            opts.folder.root      = folder["root"].as<std::string>();
            opts.folder.left      = folder["left"].as<std::string>();
            opts.folder.right     = folder["right"].as<std::string>();
            opts.folder.timescale = folder["timescale"].as<double>();
        }

        if (const auto slam = config["slam"])
        {
            if (const auto cell = slam["cell"])
            {
                opts.slam.cell_size.width  = cell["width"].as<double>();
                opts.slam.cell_size.height = cell["height"].as<double>();
            }
        }
    } catch (const YAML::Exception &e)
    {
        std::println("Failed to load config file: {}", e.what());
    }

    return opts;
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

void zenslam::options::print() const
{
    folder.print();
    slam.print();
}
