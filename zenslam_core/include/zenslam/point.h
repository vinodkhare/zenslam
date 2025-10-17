#pragma once

#include <opencv2/core/types.hpp>
#include <opencv2/viz/types.hpp>

namespace zenslam
{
    class point : public cv::Point3d
    {
    public:
        size_t    index { };
        cv::Vec3b color { };
    };
}