#include <csignal>
#include <format>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include <spdlog/spdlog.h>

#include "application.h"
#include "options.h"
#include "slam_thread.h"
#include "stereo_folder_reader.h"
#include "thread_safe.h"
#include "utils.h"

std::atomic is_running { true };

void signal_handler(const int signal)
{
    if (signal == SIGINT)
    {
        SPDLOG_INFO("CTRL+C detected, shutting down...");
        is_running = false;
    }
}

int main(const int argc, char **argv)
{
    std::signal(SIGINT, signal_handler);

    spdlog::set_level(spdlog::level::debug);

    try
    {
        const auto& options = zenslam::options::parse(argc, argv);

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

        auto application = zenslam::application { options };

        while (is_running)
        {
            application.render();
        }

        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << "\n";
        return 1;
    }
}
