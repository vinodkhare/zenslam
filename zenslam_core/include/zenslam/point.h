#pragma once

#include <opencv2/core/types.hpp>

namespace zenslam
{
    class point : public cv::Point3d
    {
    public:
        size_t index {};
    };
}
