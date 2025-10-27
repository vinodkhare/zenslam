#pragma once

#include <cstddef>

namespace zenslam::frame
{
    struct counts
    {
        size_t keypoints_l                  = { };
        size_t keypoints_r                  = { };
        size_t keypoints_l_tracked          = { };
        size_t keypoints_r_tracked          = { };
        size_t keypoints_l_new              = { };
        size_t keypoints_r_new              = { };
        size_t keypoints_total              = { };
        size_t matches                      = { };
        size_t matches_triangulated         = { };
        size_t correspondences_3d3d         = { };
        size_t correspondences_3d2d         = { };
        size_t correspondences_2d2d         = { };
        size_t correspondences_3d3d_inliers = { };
        size_t correspondences_3d2d_inliers = { };
        size_t correspondences_2d2d_inliers = { };
        size_t points                       = { };
        size_t lines                        = { };

        auto print() const -> void;
    };
}