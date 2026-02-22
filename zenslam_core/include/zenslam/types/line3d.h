#pragma once

#include "point3d.h"

namespace zenslam
{
    struct line3d : std::array<cv::Point3d, 2>
    {
        static size_t index_next;

        size_t  index      = { };
        cv::Mat descriptor = { };
    };
}

inline auto operator*(const cv::Affine3d& pose, const zenslam::line3d& line) -> zenslam::line3d
{
    auto line_trans = line;

    line_trans[0] = pose * line[0];
    line_trans[1] = pose * line[1];

    return line_trans;
}
