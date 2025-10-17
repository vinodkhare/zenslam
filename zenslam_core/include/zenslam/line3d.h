#pragma once

#include "point3d.h"

namespace zenslam
{
    struct line3d
    {
        static size_t index_next;

        size_t                     index    = { };
        std::array<cv::Point3d, 2> points3d = { };
    };
}
