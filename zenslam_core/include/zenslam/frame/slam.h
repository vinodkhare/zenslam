#pragma once

#include <array>
#include <map>
#include <vector>

#include <opencv2/core.hpp>

#include "zenslam/point3d.h"
#include "zenslam/frame/counts.h"
#include "zenslam/frame/durations.h"
#include "zenslam/frame/stereo.h"

namespace zenslam::frame
{
    class slam
    {
    public:
        std::array<stereo, 2>   frames    = { };
        std::map<size_t, point3d> points    = { };
        std::vector<cv::Vec3b>  colors    = { };
        durations               durations = { };
        counts                  counts    = { };
    };
}
