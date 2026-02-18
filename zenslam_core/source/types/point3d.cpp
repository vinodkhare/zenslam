#include "../../include/zenslam/types/point3d.h"

#include <ranges>

auto zenslam::point3d::create
(
    const std::vector<cv::Point3d>& points,
    const std::vector<size_t>&      indices,
    const std::vector<cv::Mat>&     descriptors,
    const std::vector<cv::Vec3b>&   colors
) -> std::vector<point3d>
{
    if (colors.empty())
    {
        // No colors provided, use default white
        return std::views::zip(points, indices, descriptors) | std::views::transform
        (
            [](const auto& tuple)
            {
                return point3d {std::get<0>(tuple), std::get<1>(tuple), std::get<2>(tuple), cv::Vec3b(255, 255, 255)};
            }
        ) | std::ranges::to<std::vector>();
    }
    else
    {
        // Colors provided
        return std::views::zip(points, indices, descriptors, colors) | std::views::transform
        (
            [](const auto& tuple)
            {
                return point3d {std::get<0>(tuple), std::get<1>(tuple), std::get<2>(tuple), std::get<3>(tuple)};
            }
        ) | std::ranges::to<std::vector>();
    }
}