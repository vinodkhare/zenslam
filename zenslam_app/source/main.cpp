#include <csignal>
#include <iostream>

#include <opencv2/core.hpp>

#include <spdlog/spdlog.h>

#include "application.h"
#include <zenslam/options.h>
#include <zenslam/utils.h>

#include <hello_imgui/hello_imgui.h>

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

    spdlog::set_level(spdlog::level::trace);

    try
    {
        const auto &options = zenslam::options::parse(argc, argv);

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

        spdlog::set_level(options.log_level);
        spdlog::set_pattern(options.log_pattern);

        options.print();

        auto application = zenslam::application { options };

        HelloImGui::Run
        (
            [&application]
            {
                ImGui::Text("Hello, world!");

                application.render();
            },

            // Gui code
            "ZenSLAM!",
            true,
            true,
            HelloImGui::ScreenSize({ 600, 400 })
        );

        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << "\n";
        return 1;
    }
}
