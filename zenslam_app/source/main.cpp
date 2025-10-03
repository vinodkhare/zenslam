#include <csignal>
#include <format>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>

#include <spdlog/spdlog.h>

#include "options.h"
#include "slam_thread.h"
#include "stereo_folder_reader.h"
#include "thread_safe.h"
#include "utils.h"

std::atomic<bool> is_running { true };

void signal_handler(int signal)
{
    if (signal == SIGINT)
    {
        SPDLOG_INFO("CTRL+C detected, shutting down...");
        is_running = false;
    }
}

int main(int argc, char **argv)
{
    std::signal(SIGINT, signal_handler);

    spdlog::set_level(spdlog::level::debug);

    try
    {
        auto options = zenslam::options::parse(argc, argv);

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

        auto slam_thread = zenslam::slam_thread(options);

        slam_thread.on_frame += [&stereo](const zenslam::stereo_frame &frame)
        {
            stereo = frame;
        };

        slam_thread.on_keypoints += [&keypoints_image_l, &keypoints_image_r, &matches_image]
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


        while (is_running)
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
