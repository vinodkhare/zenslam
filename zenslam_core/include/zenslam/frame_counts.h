#pragma once

#include <cstddef>

namespace zenslam
{
    struct frame_counts
    {
        size_t keypoints_l                  = { };
        size_t keypoints_r                  = { };
        size_t keypoints_l_tracked          = { };
        size_t keypoints_r_tracked          = { };
        size_t matches                      = { };
        size_t maches_triangulated          = { };
        size_t correspondences_3d3d         = { };
        size_t correspondences_3d2d         = { };
        size_t correspondences_3d3d_inliers = { };
        size_t correspondences_3d2d_inliers = { };

        auto print() const -> void;
    };
}
