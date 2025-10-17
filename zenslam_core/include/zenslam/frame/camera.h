#pragma once

#include <map>
#include <utility>

#include <opencv2/core.hpp>

#include "zenslam/keyline.h"
#include "zenslam/keypoint.h"

namespace zenslam::frame
{
    class camera
    {
    public:
        double                     timestamp   = { std::nan("nan") };
        cv::Mat                    image       = { };
        cv::Mat                    rectified   = { };
        cv::Mat                    undistorted = { };
        std::map<size_t, keypoint> keypoints   = { };
        std::map<size_t, keyline>  keylines    = { };
        std::vector<cv::Mat>       pyramid     = { };

        camera() = default;

        camera(const double timestamp, cv::Mat image) :
            timestamp(timestamp),
            image(std::move(image)) {}
    };
} // namespace zenslam::frame
