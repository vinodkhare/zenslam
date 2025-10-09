#pragma once

#include <map>
#include <utility>
#include <opencv2/core.hpp>

#include "keypoint.h"

namespace zenslam
{
    class mono_frame
    {
    public:
        double                     timestamp   = { std::nan("nan") };
        cv::Mat                    image       = { };
        cv::Mat                    rectified   = { };
        cv::Mat                    undistorted = { };
        std::map<size_t, keypoint> keypoints   = { };
        std::vector<cv::Mat>       pyramid     = { };

        mono_frame() = default;

        mono_frame(const double timestamp, cv::Mat image) :
            timestamp(timestamp),
            image(std::move(image)) {}
    };
} // namespace zenslam
