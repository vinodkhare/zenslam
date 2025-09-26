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
#include <ranges>
#include <boost/program_options.hpp>

#include "folder_options.h"
#include "mono_folder_reader.h"

namespace fs = std::filesystem;
namespace po = boost::program_options;

int main(int argc, char **argv)
{
    // Parse optional folder options using Boost.Program_options

    try
    {
        zenslam::folder_options folder_options;

        auto folder_options_description = boost::program_options::options_description("folder options");

        folder_options_description.add_options()
                ("folder-root", po::value<std::string>()->default_value(folder_options.folder_root), "Root folder")
                ("folder-left", po::value<std::string>()->default_value(folder_options.folder_left),
                 "Left folder relative to root (or absolute)")
                ("folder-right", po::value<std::string>()->default_value(folder_options.folder_right),
                 "Right folder relative to root (or absolute)")
                ("folder-timescale", po::value<double>()->default_value(folder_options.folder_timescale),
                 "Timescale for folder timestamps")
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
        if (map.contains("folder-timescale")) folder_options.folder_timescale = map["folder-timescale"].as<double>();

        folder_options.print();

        auto folder_reader_l = zenslam::mono_folder_reader(folder_options.folder_root / folder_options.folder_left, false,
                                                      folder_options.folder_timescale);

        auto folder_reader_r = zenslam::mono_folder_reader(folder_options.folder_root / folder_options.folder_right, false,
                                                      folder_options.folder_timescale);

        for (auto [frame_l, frame_r]: std::views::zip(folder_reader_l, folder_reader_r))
        {
            cv::imshow("L", frame_l.image);
            cv::imshow("R", frame_r.image);
            cv::waitKey(1);
        }
    } catch (const std::exception &e)
    {
        std::cerr << "Error parsing folder options: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
