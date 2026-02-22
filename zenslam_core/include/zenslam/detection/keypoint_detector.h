#pragma once

#include "keyline_detector.h"

#include "zenslam/all_options.h"
#include "zenslam/types/keypoint.h"
#include "zenslam/types/map.h"

namespace zenslam
{
    class keypoint_detector
    {
    public:
        keypoint_detector(const detection_options& options);

        [[nodiscard]] std::vector<keypoint> detect_keypoints(const cv::Mat& image, const map<keypoint>& keypoints_existing) const;

    private:
        detection_options      _options   = { };
        cv::Ptr<cv::Feature2D> _detector  = { }; // Underlying detector
        cv::Ptr<cv::Feature2D> _describer = { };
    };
}
