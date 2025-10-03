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
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>

#include <spdlog/spdlog.h>

#include "grid_detector.h"
#include "options.h"
#include "stereo_folder_reader.h"
#include "thread_safe.h"
#include "utils.h"
#include "viewer.h"

namespace filesystem = std::filesystem;
namespace program_options = boost::program_options;

zenslam::options parse(int argc, char **argv)
{
    zenslam::options options;
    zenslam::options options_default;

    auto map         = program_options::variables_map();
    auto description = zenslam::options::description(); // inlining this can cause argument parse failures, don't know why!
    auto parsed      = program_options::parse_command_line(argc, argv, description);

    program_options::store(parsed, map);
    program_options::notify(map);

    if (parsed.options.empty() || map.contains("help"))
    {
        options.verb = zenslam::verb::HELP;
    }

    if (map.contains("version"))
    {
        options.verb = zenslam::verb::VERSION;
    }

    auto options_map = zenslam::utils::to_map(parsed.options);

    if (options_map.contains("options-file")) options = zenslam::options::read(map["options-file"].as<std::string>());

    if (options_map.contains("folder-root")) options.folder.root = map["folder-root"].as<std::string>();
    if (options_map.contains("folder-left")) options.folder.left = map["folder-left"].as<std::string>();
    if (options_map.contains("folder-right")) options.folder.right = map["folder-right"].as<std::string>();
    if (options_map.contains("folder-timescale")) options.folder.timescale = map["folder-timescale"].as<double>();
    if (options_map.contains("options-file")) options.file = map["options-file"].as<std::string>();

    return options;
}

int main(int argc, char **argv)
{
    spdlog::set_level(spdlog::level::debug);

    try
    {
        auto options = parse(argc, argv);

        if (options.verb == zenslam::verb::HELP)
        {
            std::cout << zenslam::options::description() << "\n";
            return 0;
        }

        if (options.verb == zenslam::verb::VERSION)
        {
            std::cout << zenslam::utils::version << "\n";
            return 0;
        }

        options.print();

        zenslam::thread_safe<zenslam::stereo_frame> stereo;
        zenslam::thread_safe<cv::Mat>               keypoints_image_l;
        zenslam::thread_safe<cv::Mat>               keypoints_image_r;
        zenslam::thread_safe<cv::Mat>               matches_image;

        const auto &on_frame = [&stereo](const zenslam::stereo_frame &frame)
        {
            stereo = frame;
        };

        const auto &on_keypoints = [&keypoints_image_l, &keypoints_image_r, &matches_image]
        (const zenslam::stereo_frame &frame)
        {
            cv::Mat vis_l, vis_r;

            // DRAW_RICH_KEYPOINTS shows size and orientation
            cv::drawKeypoints
            (
                frame.l.image,
                frame.l.keypoints,
                vis_l,
                cv::Scalar(0, 255, 0),
                cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS
            );

            cv::drawKeypoints
            (
                frame.r.image,
                frame.r.keypoints,
                vis_r,
                cv::Scalar(0, 255, 0),
                cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS
            );

            keypoints_image_l = vis_l;
            keypoints_image_r = vis_r;

            // draw matches
            cv::Mat vis_matches;
            cv::drawMatches
            (
                frame.l.image,
                frame.l.keypoints,
                frame.r.image,
                frame.r.keypoints,
                frame.matches,
                vis_matches,
                cv::Scalar(0, 255, 0),
                cv::Scalar(0, 255, 0),
                std::vector<char>(),
                cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS
            );

            matches_image = vis_matches;
        };

        std::jthread slam_thread
        (
            [options, on_frame, on_keypoints]()
            {
                const auto &stereo_reader = zenslam::stereo_folder_reader
                (
                    options.folder.root / options.folder.left,
                    options.folder.root / options.folder.right,
                    options.folder.timescale
                );

                // Create a base detector (FAST)
                const auto &feature_detector  = cv::FastFeatureDetector::create(32);
                const auto &feature_describer = cv::ORB::create();
                const auto &detector          = zenslam::grid_detector::create(feature_detector, options.slam.cell_size);
                const auto &matcher           = cv::BFMatcher::create(cv::NORM_L2, false);

                auto queue = std::queue<zenslam::stereo_frame> { };

                for (auto frame: stereo_reader)
                {
                    on_frame(frame);

                    detector->detect(frame.l.image, frame.l.keypoints, cv::noArray());
                    detector->detect(frame.r.image, frame.r.keypoints, cv::noArray());

                    SPDLOG_INFO("Detected points L: {}", frame.l.keypoints.size());
                    SPDLOG_INFO("Detected points R: {}", frame.r.keypoints.size());

                    cv::Mat descriptors_l;
                    cv::Mat descriptors_r;


                    feature_describer->compute(frame.l.image, frame.l.keypoints, descriptors_l);
                    feature_describer->compute(frame.r.image, frame.r.keypoints, descriptors_r);

                    std::vector<cv::DMatch> matches = { };
                    matcher->match(descriptors_l, descriptors_r, matches);
                    SPDLOG_INFO("Matches: {}", matches.size());

                    frame.matches = matches;

                    on_keypoints(frame);
                }
            }
        );

        while (true)
        {
            // show the stereo frame
            if (!stereo->l.image.empty() && !stereo->r.image.empty())
            {
                cv::imshow("L", stereo->l.image);
                cv::setWindowTitle
                (
                    "L",
                    std::format("L: {{ t: {} }}", zenslam::utils::epoch_double_to_string(stereo->l.timestamp))
                );

                cv::imshow("R", stereo->r.image);
                cv::setWindowTitle
                (
                    "R",
                    std::format("R: {{ t: {} }}", zenslam::utils::epoch_double_to_string(stereo->r.timestamp))
                );
            }

            // display rich keypoints on the image
            if (!keypoints_image_l->empty() && !keypoints_image_r->empty())
            {
                cv::imshow("L_kp", *keypoints_image_l);
                cv::setWindowTitle("L_kp", "L_Kp");

                cv::imshow("R_kp", *keypoints_image_r);
                cv::setWindowTitle("R_kp", "R_kp");
            }

            // display matches
            if (!matches_image->empty())
            {
                cv::imshow("matches", *matches_image);
                cv::setWindowTitle("matches", "matches");
            }

            cv::waitKey(1);
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error parsing folder options: " << e.what() << "\n";
        return 1;
    }
}
