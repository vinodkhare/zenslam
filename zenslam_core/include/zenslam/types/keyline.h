#pragma once

#include <cstddef>

#include <opencv2/line_descriptor.hpp>

namespace zenslam
{
    struct keyline : cv::line_descriptor::KeyLine
    {
        static size_t index_next;

        std::size_t index      = { };
        cv::Mat     descriptor = { };
    };
} // namespace zenslam