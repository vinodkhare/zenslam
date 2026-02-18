#pragma once

#include "sensor.h"

#include "zenslam/motion/integrator_types.h"

namespace zenslam::frame
{
    struct processed : sensor
    {
        std::array<cv::Mat, 2>              undistorted = { };
        std::array<std::vector<cv::Mat>, 2> pyramids    = { };
        integral                            integral    = { };
    };
}
