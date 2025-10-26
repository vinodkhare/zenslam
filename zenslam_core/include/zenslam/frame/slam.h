#pragma once

#include "tracked.h"

namespace zenslam::frame
{
    struct slam : tracked
    {
        cv::Affine3d pose = { };
        cv::Affine3d pose_gt = { };
    };
}
