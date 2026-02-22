#include <csignal>
#include <iostream>

#include <hello_imgui/hello_imgui.h>

#include <opencv2/core.hpp>

#include <spdlog/spdlog.h>

#include <zenslam/all_options.h>
#include <zenslam/options_parser.h>
#include <zenslam/options_printer.h>
#include <zenslam/utils/utils.h>

#include "application.h"

std::atomic is_running { true };

void signal_handler(const int signal)
{
    if (signal == SIGINT)
    {
        SPDLOG_INFO("CTRL+C detected, shutting down...");
        is_running = false;
    }
}

int main(const int argc, char** argv)
{
    std::signal(SIGINT, signal_handler);

    spdlog::set_level(spdlog::level::trace);

    try
    {
        // Parse command line arguments to get options file path
        std::filesystem::path options_file = "options.yaml";
        bool show_help = false;
        bool show_version = false;

        for (int i = 1; i < argc; ++i)
        {
            std::string arg = argv[i];
            if (arg == "--options-file" || arg == "-o")
            {
                if (i + 1 < argc)
                    options_file = argv[++i];
            }
            else if (arg == "--help" || arg == "-h")
            {
                show_help = true;
            }
            else if (arg == "--version" || arg == "-v")
            {
                show_version = true;
            }
        }

        if (show_help)
        {
            std::cout << "ZenSLAM - Stereo Visual-Inertial SLAM\n"
                      << "Usage: zenslam_app [OPTIONS]\n"
                      << "Options:\n"
                      << "  --options-file PATH  Path to YAML configuration file (default: options.yaml)\n"
                      << "  --help, -h           Show this help message\n"
                      << "  --version, -v        Show version information\n";
            return 0;
        }

        if (show_version)
        {
            std::cout << zenslam::utils::version << "\n";
            return 0;
        }

        // Load options from YAML file
        auto options = zenslam::options_parser::load(options_file);

        spdlog::set_level(options.log_level);
        spdlog::set_pattern(options.log_pattern);

        zenslam::options_printer::print(options);

        zenslam::application     application { options };
        HelloImGui::RunnerParams params { };

        // Force Metal backend
        params.rendererBackendType = HelloImGui::RendererBackendType::Metal;
        params.callbacks.ShowGui   = [&application]
        {
            application.render();
        };

        HelloImGui::Run(params);

        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << "\n";
        return 1;
    }
}
