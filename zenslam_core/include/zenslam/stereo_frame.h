#pragma once

#include <map>
#include <utility>

#include <opencv2/core/affine.hpp>

#include "match_data.h"
#include "mono_frame.h"
#include "point.h"

namespace zenslam
{
    class stereo_frame
    {
    public:
        mono_frame               l        = { };
        mono_frame               r        = { };
        match_data               spatial  = { };
        match_data               temporal = { };
        cv::Affine3d             pose     = { cv::Affine3d::Identity() };
        std::vector<cv::Point3d> points3d = { };
        std::map<size_t, point>  points   = { };

        stereo_frame() = default;

        stereo_frame(mono_frame l, mono_frame r) :
            l(std::move(l)),
            r(std::move(r)) {}

        stereo_frame(const stereo_frame &other)                = default;
        stereo_frame &operator=(const stereo_frame &other)     = default;
        stereo_frame(stereo_frame &&other) noexcept            = default;
        stereo_frame &operator=(stereo_frame &&other) noexcept = default;
    };
} // namespace zenslam
