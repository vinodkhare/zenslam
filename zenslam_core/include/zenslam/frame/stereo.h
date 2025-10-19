#pragma once

#include <map>
#include <utility>

#include <opencv2/core/affine.hpp>

#include "zenslam/line3d.h"
#include "zenslam/point3d.h"
#include "zenslam/frame/camera.h"

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

        stereo(camera l, camera r) :
            cameras { std::move(l), std::move(r) } {}

        stereo()                                   = default;
        stereo(const stereo& other)                = default;
        stereo(stereo&& other) noexcept            = default;
        stereo& operator=(const stereo& other)     = default;
        stereo& operator=(stereo&& other) noexcept = default;
    };
} // namespace zenslam::frame