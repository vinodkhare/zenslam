#pragma once

#include "sensor.h"

#include <preint/preint.h>

namespace zenslam::frame
{
    struct processed : sensor
    {
        std::array<cv::Mat, 2>              undistorted = { };
        std::array<std::vector<cv::Mat>, 2> pyramids    = { };
        ugpm::PreintMeas                    preint      = { };
    };
}