#pragma once

#include <vector>

#include <opencv2/core/types.hpp>

namespace zenslam
{
    class match_data
    {
    public:
        std::vector<cv::DMatch> matches{};
        std::vector<cv::DMatch> filtered{};
        std::vector<cv::DMatch> unmatched{};
    };
} // namespace zenslam
