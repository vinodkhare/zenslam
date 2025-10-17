#pragma once

#include <map>
#include <utility>

#include <opencv2/core/affine.hpp>

#include "camera_frame.h"
#include "point.h"

namespace zenslam
{
    class stereo_frame
    {
    public:
        std::array<camera_frame, 2> cameras = { };
        std::map<size_t, point>     points  = { };
        cv::Affine3d                pose    = { };
        cv::Affine3d                pose_gt = { }; // groundtruth pose

        stereo_frame(camera_frame l, camera_frame r) :
            cameras { std::move(l), std::move(r) } {}

        stereo_frame()                                         = default;
        stereo_frame(const stereo_frame &other)                = default;
        stereo_frame(stereo_frame &&other) noexcept            = default;
        stereo_frame &operator=(const stereo_frame &other)     = default;
        stereo_frame &operator=(stereo_frame &&other) noexcept = default;
    };
} // namespace zenslam
