#pragma once

#include "zenslam/frame/counts.h"
#include "zenslam/frame/durations.h"
#include "zenslam/frame/estimated.h"

namespace zenslam::frame
{
    struct system : std::array<estimated, 2>
    {
        point3d_cloud points3d  = { };
        map<line3d>   lines3d   = { };
        counts        counts    = { };
        durations     durations = { };
    };
}