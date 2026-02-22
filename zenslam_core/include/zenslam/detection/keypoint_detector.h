#pragma once

#include "zenslam/types/keypoint.h"
#include "zenslam/types/map.h"

namespace zenslam
{
    class keypoint_detector
    {
    public:
        virtual ~keypoint_detector() = default;

        [[nodiscard]] virtual std::vector<keypoint> detect_keypoints(const cv::Mat& image, const map<keypoint>& keypoints_existing) const = 0;
    };
}
