#pragma once

#include <array>

#include "stereo_frame.h"

namespace zenslam
{
    class slam_frame
    {
    public:
        std::array<stereo_frame, 2> frame  = { };
        std::map<size_t, point>     points = { };
        std::vector<cv::Vec3b>      colors = { };
    };
}
