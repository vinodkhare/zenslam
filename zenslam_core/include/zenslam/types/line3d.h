#pragma once

#include "point3d.h"

namespace zenslam
{
    struct line3d : std::array<cv::Point3d, 2>
    {
        static size_t index_next;

        size_t index = { };
    };
}