#pragma once

#include <utility>

#include <opencv2/core.hpp>

#include "zenslam/types/keyline.h"
#include "zenslam/types/keypoint.h"
#include "zenslam/types/map.h"

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
    };
} // namespace zenslam::frame
