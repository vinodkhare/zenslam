#pragma once

#include "tracked.h"

namespace zenslam::frame
{
    struct estimated : tracked
    {
        cv::Affine3d pose        = { };
        cv::Affine3d pose_gt     = { };
        bool         is_keyframe = false;
    };
}
