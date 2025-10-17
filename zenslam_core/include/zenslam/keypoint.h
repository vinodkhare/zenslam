#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

namespace zenslam
{
    class keypoint : public cv::KeyPoint
    {
    public:
        static size_t index_next;

        size_t  index { };
        cv::Mat descriptor { };
    };
}