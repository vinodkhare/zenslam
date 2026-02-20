#pragma once

#include "zenslam/options_base.h"

namespace zenslam
{
    /// 3D-3D rigid transformation configuration
    class rigid_options : public options_base<rigid_options, "rigid options", "rigid">
    {
    public:
        ZENSLAM_DEFINE_OPTIONS(
            ((double, threshold, 0.1, "3D-3D rigid transformation RANSAC threshold in meters"))
            ((int, iterations, 1000, "3D-3D rigid transformation RANSAC maximum iterations"))
            ((int, min_correspondences, 3, "3D-3D rigid transformation minimum correspondences"))
        )
    };
} // namespace zenslam
