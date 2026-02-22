#pragma once

#include <opencv2/features2d.hpp>

#include "keypoint_detector.h"

#include "zenslam/all_options.h"
#include "zenslam/types/keypoint.h"
#include "zenslam/types/map.h"

namespace zenslam
{
    class keypoint_detector_parallel : public keypoint_detector
    {
    public:
        keypoint_detector_parallel(const detection_options& options);

        /* Detect keypoints in the image using parallel grid-based detection
         *
         * This is a parallel version of detect_keypoints that processes grid cells concurrently
         * using std::async. It provides better performance on multi-core systems when detecting
         * features across many grid cells.
         *
         * @param image The input image in which to detect keypoints
         * @param keypoints_existing A map to store detected keypoints with their indices
         * @return A vector of detected keypoints
         */
        [[nodiscard]] std::vector<keypoint> detect_keypoints(const cv::Mat& image, const map<keypoint>& keypoints_existing) const override;

    private:
        detection_options      _options   = { };
        cv::Ptr<cv::Feature2D> _detector  = { }; // Underlying detector
        cv::Ptr<cv::Feature2D> _describer = { };
    };
}
