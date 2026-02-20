#pragma once

#include "zenslam/options_base.h"

namespace zenslam
{
    /// PnP RANSAC estimation configuration
    class pnp_options : public options_base<pnp_options, "pnp options", "pnp">
    {
    public:
        ZENSLAM_DEFINE_OPTIONS(
            ((int, iterations, 1000, "PnP RANSAC maximum iterations"))
            ((float, threshold, 3.0f, "PnP RANSAC inlier threshold in pixels"))
            ((double, confidence, 0.99, "PnP RANSAC confidence level (0.0-1.0)"))
            ((bool, use_refinement, true, "Use iterative LM refinement after RANSAC pose estimation"))
            ((int, min_refinement_inliers, 4, "Minimum inliers required for PnP refinement"))
        )
    };
} // namespace zenslam
