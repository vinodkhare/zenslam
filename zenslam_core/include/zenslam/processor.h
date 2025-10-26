#pragma once

#include <opencv2/imgproc.hpp>

#include "calibration.h"
#include "options.h"

#include "frame/processed.h"
#include "frame/sensor.h"

namespace zenslam
{
    class processor
    {
    public:
        explicit processor(class options::slam options, const zenslam::calibration& calibration);

        auto process(const frame::sensor& sensor) const -> frame::processed;

    private:
        cv::Ptr<cv::CLAHE>  _clahe       = cv::createCLAHE(4.0); // TODO: make configurable
        class options::slam _options     = { };
        calibration         _calibration = { };
    };
}
