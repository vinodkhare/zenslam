#pragma once

#include <filesystem>

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>

#include <opencv2/core/types.hpp>

#include "verb.h"

namespace zenslam
{
    class options
    {
    public:
        using options_description = boost::program_options::options_description;

        static options_description description();

        static options parse(int argc, char **argv);
        static options parse(const std::filesystem::path &path);

        std::filesystem::path file = { "options.yaml" };
        verb                  verb = { verb::RUN };

        class folder
        {
        public:
            static options_description description();

            std::filesystem::path root      = { "." };
            std::filesystem::path left      = { "cam0" };
            std::filesystem::path right     = { "cam1" };
            double                timescale = { 1.0 };

            void print() const;
        } folder;

        class slam
        {
        public:
            cv::Size cell_size { 16, 16 };

            void print() const;
        } slam;

        void print() const;
    };
}
