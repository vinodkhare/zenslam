#pragma once

#include <cmath>
#include <opencv2/imgproc.hpp>

#include "zenslam/calibration/calibration.h"
#include "zenslam/motion/integrator.h"
#include "options.h"

#include "frame/processed.h"
#include "frame/sensor.h"

namespace zenslam
{
    class processor
    {
    public:
        explicit processor(slam_options options, calibration calibration);

        /**
         * @brief Processes sensor frame data and returns a processed frame.
         *
         * This function takes raw sensor frame data as input and performs necessary
         * processing operations to convert it into a processed frame suitable for
         * further use in the SLAM pipeline. It enhances the image using CLAHE if
         * enabled in the options, rectifies the images, builds image pyramids, and pre-integrates
         * IMU data.
         *
         * @param sensor The input sensor frame containing raw data to be processed.
         * @return frame::processed The processed frame result after applying processing operations.
         */
        auto process(const frame::sensor& sensor) -> frame::processed;

    private:
        cv::Ptr<cv::CLAHE> _clahe       = cv::createCLAHE(4.0); // TODO: make configurable
        slam_options       _options     = { };
        calibration        _calibration = { };
        integrator         _integrator  = integrator { _calibration.imu, _options.integrator_method };
        double             _timestamp   = std::nan("nan");
    };
} // namespace zenslam
