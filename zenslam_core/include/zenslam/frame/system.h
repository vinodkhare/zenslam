#pragma once

#include "zenslam/frame/counts.h"
#include "zenslam/frame/durations.h"
#include "zenslam/frame/estimated.h"
#include "zenslam/types/line3d_cloud.h"
#include "zenslam/types/point3d_cloud.h"

namespace zenslam::frame
{
    struct system : std::array<estimated, 2>
    {
        point3d_cloud points3d  = { };
        line3d_cloud  lines3d   = { };
        counts        counts    = { };
        durations     durations = { };
    };
}
