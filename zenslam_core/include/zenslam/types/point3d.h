#pragma once


#include <opencv2/core/types.hpp>
#include <opencv2/viz/types.hpp>

#include "map.h"

namespace zenslam
{
    class point3d : public cv::Point3d
    {
    public:
        static auto create
        (
            const std::vector<cv::Point3d>& points,
            const std::vector<size_t>&      indices,
            const std::vector<cv::Mat>&     descriptors
        ) -> std::vector<point3d>;

        size_t  index      = { };
        cv::Mat descriptor = { };
    };
}

inline auto operator*(const cv::Affine3d& pose, const std::vector<zenslam::point3d>& points3d) -> std::vector<zenslam::point3d>
{
    std::vector<zenslam::point3d> points3d_trans = { };
    points3d_trans.reserve(points3d.size());

    for (const auto& point3d : points3d)
    {
        points3d_trans.emplace_back(pose * point3d, point3d.index, point3d.descriptor);
    }

    return points3d_trans;
}

inline auto operator*(const cv::Affine3d& pose, const zenslam::map<zenslam::point3d>& points3d) -> zenslam::map<zenslam::point3d>
{
    zenslam::map<zenslam::point3d> points3d_trans { };

    for (const auto& point3d : points3d | std::views::values)
    {
        points3d_trans[point3d.index]            = pose * point3d;
        points3d_trans[point3d.index].index      = point3d.index;
        points3d_trans[point3d.index].descriptor = point3d.descriptor;
    }

    return points3d_trans;
}