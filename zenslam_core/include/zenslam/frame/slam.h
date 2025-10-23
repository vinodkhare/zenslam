#pragma once

#include "zenslam/frame/tracked.h"

namespace zenslam::frame
{
    struct slam : tracked
    {
        cv::Affine3d pose = { };
        cv::Affine3d pose_gt = { };
    };
}
