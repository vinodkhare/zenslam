#pragma once

#include <filesystem>

#include <opencv2/core/types.hpp>

namespace zenslam
{
    class options
    {
    public:
        static options read(const std::filesystem::path &path);

        class folder
        {
        public:
            std::filesystem::path root      = ".";
            std::filesystem::path left      = "cam0";
            std::filesystem::path right     = "cam1";
            double                timescale = 1E-9;

            void print() const;
        } folder;

        class slam
        {
        public:
            cv::Size cell_size = { 16, 16 };

            void print() const;
        } slam;

        std::filesystem::path options_file = "options.yaml";

        void print() const;
    };
}
