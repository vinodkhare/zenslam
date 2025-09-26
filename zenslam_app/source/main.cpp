// Stereo image directory viewer
// Usage:
//   zenslam_app <left_dir> <right_dir> [--delay ms] [--loop] [--resize WIDTH HEIGHT]
// Images in each directory are expected to be individually loadable by OpenCV (png, jpg, etc.).
// Frames are paired by sorted filename order. Extra frames in the longer sequence are ignored.

#include <chrono>
#include <filesystem>
#include <format>
#include <iostream>
#include <ranges>
#include <string>
#include <vector>

#include <boost/program_options.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>

#include <spdlog/spdlog.h>

#include "folder_options.h"
#include "stereo_folder_reader.h"

namespace filesystem = std::filesystem;
namespace program_options = boost::program_options;

namespace zenslam
{
    std::string epoch_double_to_string(const double epoch_seconds)
    {
        // Split into integral seconds and fractional milliseconds
        const auto &seconds      = floor<std::chrono::seconds>(std::chrono::duration<double>(epoch_seconds));
        const auto &milliseconds = duration_cast<std::chrono::milliseconds>
                (std::chrono::duration<double>(epoch_seconds) - seconds);

        // sys_time with milliseconds precision
        auto time_point = std::chrono::sys_seconds(seconds) + milliseconds;

        // Format: YYYY-MM-DD HH:MM:SS.mmm UTC
        return std::format("{:%F %T} UTC", time_point);
    }
}

int main(int argc, char **argv)
{
    spdlog::set_level(spdlog::level::debug);

    try
    {
        zenslam::folder_options folder_options;

        auto folder_options_description = program_options::options_description("folder options");

        folder_options_description.add_options()
                (
                    "folder-root",
                    program_options::value<std::string>()->default_value(folder_options.folder_root),
                    "Root folder"
                )
                (
                    "folder-left",
                    program_options::value<std::string>()->default_value(folder_options.folder_left),
                    "Left folder relative to root (or absolute)"
                )
                (
                    "folder-right",
                    program_options::value<std::string>()->default_value(folder_options.folder_right),
                    "Right folder relative to root (or absolute)"
                )
                (
                    "folder-timescale",
                    program_options::value<double>()->default_value(folder_options.folder_timescale),
                    "Timescale for folder timestamps"
                )
                ("help,h", "Show help");

        auto options_description = program_options::options_description("options");
        options_description.add(folder_options_description);

        auto map = program_options::variables_map();

        auto parsed = program_options::command_line_parser(argc, argv)
                      .options(options_description)
                      .allow_unregistered()
                      .run();

        program_options::store(parsed, map);
        program_options::notify(map);

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

        auto stereo_reader = zenslam::stereo_folder_reader
        (
            folder_options.folder_root / folder_options.folder_left,
            folder_options.folder_root / folder_options.folder_right,
            folder_options.folder_timescale
        );

        for (const auto &stereo: stereo_reader)
        {
            // show the stereo frame
            {
                cv::imshow("L", stereo.l.image);
                cv::setWindowTitle
                        ("L", std::format("L: {{ t: {} }}", zenslam::epoch_double_to_string(stereo.l.timestamp)));

                cv::imshow("R", stereo.r.image);
                cv::setWindowTitle
                        ("R", std::format("R: {{ t: {} }}", zenslam::epoch_double_to_string(stereo.r.timestamp)));

                cv::waitKey(1);
            }

            // detect keypoints
            auto keypoints_l = std::vector<cv::KeyPoint>();
            auto keypoints_r = std::vector<cv::KeyPoint>();

            auto detector = cv::FastFeatureDetector::create(16);

            detector->detect(stereo.l.image, keypoints_l);
            detector->detect(stereo.r.image, keypoints_r);

            SPDLOG_INFO("Detected points L: {}", keypoints_l.size());
            SPDLOG_INFO("Detected points R: {}", keypoints_r.size());

            // display rich keypoints on image
            {
                cv::Mat vis_l, vis_r;
                // DRAW_RICH_KEYPOINTS shows size & orientation
                cv::drawKeypoints
                (
                    stereo.l.image,
                    keypoints_l,
                    vis_l,
                    cv::Scalar(0, 255, 0),
                    cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS
                );

                cv::drawKeypoints
                (
                    stereo.r.image,
                    keypoints_r,
                    vis_r,
                    cv::Scalar(0, 255, 0),
                    cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS
                );

                cv::imshow("L_kp", vis_l);
                cv::setWindowTitle("L_kp", std::format("L_kp: {}/{} keypoints", keypoints_l.size(), 128));

                cv::imshow("R_kp", vis_r);
                cv::setWindowTitle("R_kp", std::format("R_kp: {}/{} keypoints", keypoints_r.size(), 128));

                // brief wait to allow windows to refresh without blocking playback heavily
                cv::waitKey(1);
            }
        }
    } catch (const std::exception &e)
    {
        std::cerr << "Error parsing folder options: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
