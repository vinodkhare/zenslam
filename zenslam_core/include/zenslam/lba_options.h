#pragma once

#include "zenslam/options_base.h"

namespace zenslam
{
    /// Local bundle adjustment configuration
    class lba_options : public options_base<lba_options, "lba options", "lba.">
    {
    public:
        ZENSLAM_DEFINE_OPTIONS(
            ((int, max_iterations, 30, "LBA maximum solver iterations"))
            ((double, huber_delta, 1.0, "LBA Huber loss delta in pixels"))
            ((bool, refine_landmarks, true, "Optimize landmark positions during LBA"))
        )
    };
} // namespace zenslam
