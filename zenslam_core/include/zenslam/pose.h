#pragma once

#include <cmath>
#include <filesystem>

#include <opencv2/core/affine.hpp>

namespace zenslam
{
    class pose
    {
    public:
        double       timestamp { std::nan("nan") };
        cv::Affine3d affine { cv::Affine3d::Identity() };
    };
}
