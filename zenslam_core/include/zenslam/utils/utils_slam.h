#pragma once

// Unified header for SLAM utilities - includes all specialized modules
// This header maintains backward compatibility while organizing code into logical modules

#include <map>
#include <ranges>

#include "zenslam/matching/matching_utils.h"
#include "zenslam/types/line3d.h"
#include "zenslam/types/point3d.h"

// Inline operators for geometric transformations
inline auto operator-
(
    const std::vector<cv::Point2d>& lhs,
    const std::vector<cv::Point2f>& rhs
)
    -> std::vector<cv::Point2d>
{
    std::vector<cv::Point2d> difference;
    difference.reserve(lhs.size());

    for (auto i = 0; i < lhs.size(); ++i) { difference.emplace_back(lhs[i] - cv::Point2d(rhs[i].x, rhs[i].y)); }

    return difference;
}

inline auto operator*(const cv::Affine3d& pose, const zenslam::line3d& line) -> zenslam::line3d
{
    zenslam::line3d transformed_line;

    transformed_line.index = line.index;
    transformed_line[0]    = pose * line[0];
    transformed_line[1]    = pose * line[1];

    return transformed_line;
}

inline auto operator*(const cv::Affine3d& pose, const zenslam::map<zenslam::line3d>& lines) -> zenslam::map<zenslam::line3d>
{
    zenslam::map<zenslam::line3d> transformed_lines{};

    for (const auto& line : lines | std::views::values)
    {
        transformed_lines[line.index]       = pose * line;
        transformed_lines[line.index].index = line.index;
    }

    return transformed_lines;
}
