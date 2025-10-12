#pragma once

#include <cmath>

#include <opencv2/core/quaternion.hpp>

namespace zenslam
{
    class pose
    {
    public:
        double    timestamp { std::nan("nan") };
        cv::Vec3d translation;
        cv::Quatd quaternion;
    };
}