#pragma once

#include <opencv2/core/types.hpp>
#include <opencv2/viz/types.hpp>

namespace zenslam
{
    class point3d : public cv::Point3d
    {
    public:
        static auto create(const std::vector<cv::Point3d>& points, const std::vector<size_t>& indices) -> std::vector<point3d>;

        size_t index { };
    };
}
