#include "point3d.h"

#include <ranges>

auto zenslam::point3d::create(const std::vector<size_t> &indices, const std::vector<cv::Point3d> &points) -> std::vector<point3d>
{
    return std::views::zip(indices, points) | std::views::transform
    (
        [](const auto &tuple)
        {
            const auto &[i, p] = tuple;
            return point3d { p, i};
        }
    ) | std::ranges::to<std::vector>();
}
