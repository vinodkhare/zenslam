#pragma once

#include <utility>
#include <opencv2/core.hpp>

namespace zenslam
{
    class mono_frame
    {
    public:
        double                    timestamp = { std::nan("nan") };
        cv::Mat                   image     = { };
        std::vector<cv::KeyPoint> keypoints = { };

        mono_frame() = default;

        mono_frame(const double timestamp, cv::Mat image) :
            timestamp(timestamp),
            image(std::move(image))
        {
        }
    };
} // namespace zenslam
