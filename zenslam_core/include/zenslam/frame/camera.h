#pragma once

#include <utility>

#include <opencv2/core.hpp>

#include "zenslam/keyline.h"
#include "zenslam/keypoint.h"
#include "zenslam/map.h"

namespace zenslam::frame
{
    class camera
    {
    public:
        double               timestamp   = { std::nan("nan") };
        cv::Mat              image       = { };
        cv::Mat              rectified   = { };
        cv::Mat              undistorted = { };
        std::vector<cv::Mat> pyramid     = { };
        map<keypoint>        keypoints   = { };
        map<keyline>         keylines    = { };

        camera() = default;

        camera(const double timestamp, cv::Mat image) :
            timestamp(timestamp),
            image(std::move(image)) {}
    };
} // namespace zenslam::frame