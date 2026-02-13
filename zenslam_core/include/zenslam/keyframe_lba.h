#pragma once

#include <cstddef>

#include "zenslam/calibration.h"
#include "zenslam/keyframe_database.h"
#include "zenslam/slam_options.h"

namespace zenslam
{
    struct keyframe_lba_result
    {
        bool   converged = false;
        size_t keyframes = 0;
        size_t residuals = 0;
    };

    class keyframe_lba
    {
    public:
        explicit keyframe_lba(calibration calib, lba_options options = {});

        [[nodiscard]] auto optimize(keyframe_database& db) -> keyframe_lba_result;

    private:
        calibration _calibration = { };
        lba_options _options = { };
    };
}
