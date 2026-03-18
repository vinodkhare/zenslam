#pragma once

#include <vector>

#include <opencv2/core.hpp>

namespace zenslam::metal
{
    auto is_available() -> bool;

    void calc_optical_flow_pyr_lk(
        const std::vector<cv::Mat>& prev_pyramid,
        const std::vector<cv::Mat>& next_pyramid,
        const std::vector<cv::Point2f>& prev_points,
        std::vector<cv::Point2f>& next_points,
        std::vector<uchar>& status,
        std::vector<float>& err,
        cv::Size win_size,
        int max_level,
        cv::TermCriteria criteria,
        int flags,
        double min_eig_threshold = 1e-4);
}
