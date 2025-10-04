#pragma once

#include <opencv2/core/types.hpp>

namespace zenslam
{
    class keypoint : public cv::KeyPoint
    {
    public:
        size_t index { };
    };
}
