#include "point3d.h"

#include <ranges>

auto zenslam::point3d::create
(
    const std::vector<cv::Point3d>& points,
    const std::vector<size_t>&      indices
) -> std::vector<point3d>
{
    return std::views::zip(points, indices) | std::views::transform
           (
               [](const auto& tuple)
               {
                   return point3d { std::get<0>(tuple), std::get<1>(tuple) };
               }
           ) | std::ranges::to<std::vector>();
}
