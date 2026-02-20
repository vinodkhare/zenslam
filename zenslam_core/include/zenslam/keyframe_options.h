#pragma once

#include "zenslam/options_base.h"

namespace zenslam
{
    /// Keyframe selection configuration
    class keyframe_options : public options_base<keyframe_options, "keyframe options", "keyframe">
    {
    public:
        ZENSLAM_DEFINE_OPTIONS(
            ((int, min_frames, 5, "Minimum frames between keyframes"))
            ((int, max_frames, 30, "Maximum frames between keyframes"))
            ((double, min_translation, 0.15, "Minimum translation (m) to trigger a keyframe"))
            ((double, min_rotation_deg, 10.0, "Minimum rotation (deg) to trigger a keyframe"))
            ((double, min_tracked_ratio, 0.6, "Minimum tracked ratio before forcing a keyframe"))
            ((int, min_inliers, 20, "Minimum inliers before forcing a keyframe"))
        )
    };
} // namespace zenslam
