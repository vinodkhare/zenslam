#pragma once

#include "zenslam/options_base.h"

namespace zenslam
{
    /// Essential matrix estimation configuration
    class essential_options : public options_base<essential_options, "essential options", "essential.">
    {
    public:
        ZENSLAM_DEFINE_OPTIONS(
            ((double, confidence, 0.999, "Essential matrix RANSAC confidence level (0.0-1.0)"))
            ((double, threshold, 1.0, "Essential matrix RANSAC threshold in pixels"))
            ((int, min_inliers, 5, "Essential matrix minimum inliers"))
        )
    };
} // namespace zenslam
