#pragma once

#include <filesystem>

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>

#include <opencv2/core/types.hpp>

#include "verb.h"

namespace zenslam
{
    class options
    {
    public:
        static boost::program_options::options_description description();
        static options                                     read(const std::filesystem::path &path);
        static options                                     read(const boost::program_options::variables_map &map);

        std::filesystem::path file = { "options.yaml" };
        verb                  verb = { verb::RUN };

        class folder
        {
        public:
            static boost::program_options::options_description description();

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
