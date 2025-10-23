#pragma once

#include <utility>

#include <opencv2/core/affine.hpp>

#include "zenslam/frame/camera.h"
#include "zenslam/types/line3d.h"
#include "zenslam/types/point3d.h"

namespace zenslam::frame
{
    class stereo
    {
    public:
        std::array<camera, 2> cameras  = { };
        map<point3d>          points3d = { };
        map<line3d>           lines3d  = { };
        cv::Affine3d          pose     = { };
        cv::Affine3d          pose_gt  = { }; // groundtruth pose
    };
} // namespace zenslam::frame
