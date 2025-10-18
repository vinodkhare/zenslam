#pragma once

#include <array>
#include <vector>

#include "zenslam/point3d.h"
#include "zenslam/frame/counts.h"
#include "zenslam/frame/durations.h"
#include "zenslam/frame/stereo.h"

namespace zenslam::frame
{
    class slam
    {
    public:
        std::array<stereo, 2>  frames       = { };
        map<point3d>           points3d_map = { };
        map<line3d>            lines3d_map  = { };
        std::vector<cv::Vec3b> colors       = { };
        durations              durations    = { };
        counts                 counts       = { };
    };
}
