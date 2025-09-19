// Stereo image directory viewer
// Usage:
//   zenslam_app <left_dir> <right_dir> [--delay ms] [--loop] [--resize WIDTH HEIGHT]
// Images in each directory are expected to be individually loadable by OpenCV (png, jpg, etc.).
// Frames are paired by sorted filename order. Extra frames in the longer sequence are ignored.

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <filesystem>
#include <vector>
#include <string>
#include <iostream>
#include <algorithm>
#include <boost/program_options.hpp>

#include "folder_options.h"

namespace fs = std::filesystem;
namespace po = boost::program_options;

struct options
{
    fs::path left_dir;
    fs::path right_dir;
    int      delay_ms = 30; // inter-frame delay
    bool     loop     = false;
    int      resize_w = 0; // 0 = no resize
    int      resize_h = 0;
};

bool parse_args(int argc, char **argv, options &opts)
{
    if (argc < 3)
    {
        std::cerr << "Usage: " << argv[0] << " <left_dir> <right_dir> [--delay ms] [--loop] [--resize W H]\n";
        return false;
    }
    opts.left_dir  = argv[1];
    opts.right_dir = argv[2];
    for (int i = 3; i < argc; ++i)
    {
        std::string a = argv[i];
        if (a == "--delay" && i + 1 < argc)
        {
            opts.delay_ms = std::stoi(argv[++i]);
        } else if (a == "--loop")
        {
            opts.loop = true;
        } else if (a == "--resize" && i + 2 < argc)
        {
            opts.resize_w = std::stoi(argv[++i]);
            opts.resize_h = std::stoi(argv[++i]);
        } else
        {
            std::cerr << "Unknown or incomplete argument: " << a << "\n";
            return false;
        }
    }
    if (!fs::exists(opts.left_dir) || !fs::is_directory(opts.left_dir))
    {
        std::cerr << "Left directory invalid: " << opts.left_dir << "\n";
        return false;
    }
    if (!fs::exists(opts.right_dir) || !fs::is_directory(opts.right_dir))
    {
        std::cerr << "Right directory invalid: " << opts.right_dir << "\n";
        return false;
    }
    return true;
}

std::vector<fs::path> list_images(const fs::path &dir)
{
    static const std::vector<std::string> exts{".png", ".jpg", ".jpeg", ".bmp", ".tiff"};
    std::vector<fs::path>                 files;
    for (auto &p: fs::directory_iterator(dir))
    {
        if (!p.is_regular_file()) continue;
        auto ext = p.path().extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
        if (std::find(exts.begin(), exts.end(), ext) != exts.end())
        {
            files.push_back(p.path());
        }
    }
    std::sort(files.begin(), files.end());
    return files;
}

int main(int argc, char **argv)
{
    // Parse optional folder options using Boost.Program_options

    try
    {
        zenslam::folder_options folder_options;

        auto folder_options_description = boost::program_options::options_description("folder options");

        folder_options_description.add_options()
                ("folder-root", po::value<std::string>()->default_value(folder_options.folder_root), "Root folder")
                ("folder-left", po::value<std::string>()->default_value(folder_options.folder_left), "Left folder relative to root (or absolute)")
                ("folder-right", po::value<std::string>()->default_value(folder_options.folder_right), "Right folder relative to root (or absolute)")
                ("help,h", "Show help");

        auto options_description = boost::program_options::options_description("options");
        options_description.add(folder_options_description);

        auto map = po::variables_map();

        auto parsed = po::command_line_parser(argc, argv)
                      .options(options_description)
                      .allow_unregistered()
                      .run();

        po::store(parsed, map);
        po::notify(map);

        if (parsed.options.empty() || map.contains("help"))
        {
            std::cout << options_description << "\n";
            return 0;
        }

        if (map.contains("folder-root")) folder_options.folder_root = map["folder-root"].as<std::string>();
        if (map.contains("folder-left")) folder_options.folder_left = map["folder-left"].as<std::string>();
        if (map.contains("folder-right")) folder_options.folder_right = map["folder-right"].as<std::string>();
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error parsing folder options: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
