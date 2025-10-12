#pragma once
#include <opencv2/core/affine.hpp>

namespace zenslam
{
    struct pose_data
    {
        cv::Affine3d        pose     = { cv::Affine3d::Identity() };
        std::vector<size_t> indices  = { };
        std::vector<size_t> inliers  = { };
        std::vector<size_t> outliers = { };
        std::vector<double> errors   = { };
    };
}
