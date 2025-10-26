#pragma once

#include "zenslam/frame/processed.h"
#include "zenslam/types/keyline.h"
#include "zenslam/types/keypoint.h"
#include "zenslam/types/line3d.h"
#include "zenslam/types/map.h"
#include "zenslam/types/point3d_cloud.h"

namespace zenslam::frame
{
    struct tracked : processed
    {
        std::array<map<keypoint>, 2> keypoints = { };
        std::array<map<keyline>, 2>  keylines  = { };
        map<point3d>                 points3d  = { };
        map<line3d>                  lines3d   = { };
    };
}