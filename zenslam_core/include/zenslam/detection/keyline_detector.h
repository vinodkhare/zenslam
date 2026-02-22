#pragma once

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/line_descriptor/descriptor.hpp>

#include "zenslam/all_options.h"
#include "zenslam/types/keyline.h"
#include "zenslam/types/keypoint.h"
#include "zenslam/types/map.h"

namespace zenslam
{
    /**
     * @brief Feature detector that divides an image into grid cells and detects one feature per cell
     *
     * This detector wraps any OpenCV feature detector and applies it to each grid cell
     * independently, returning the strongest keypoint from each cell. This ensures
     * a more uniform distribution of keypoints across the image.
     */
    class keyline_detector final
    {
    public:
        keyline_detector(const detection_options& options);

        /**
         * @brief Detect keylines in the image, avoiding areas where keylines already exist
         *
         * This overload creates a mask to prevent detection in regions occupied by existing
         * keylines (e.g., those tracked from the previous frame). A rectangular region around
         * each existing keyline is masked out to avoid redundant detections.
         *
         * @param image The input image in which to detect keylines
         * @param keylines_map A map of existing keylines (e.g., tracked from previous frame)
         * @param mask_margin The margin (in pixels) around each existing keyline to mask out
         * @return A vector of newly detected keylines
         */
        [[nodiscard]] std::vector<keyline> detect_keylines(const cv::Mat& image, const map<keyline>& keylines_map, int mask_margin = 10) const;

    private:
        detection_options _options = { };

        cv::Ptr<cv::line_descriptor::LSDDetector>      _detector  = { }; // Line feature detector
        cv::Ptr<cv::line_descriptor::BinaryDescriptor> _describer = { }; // Line descriptor extractor
    };
} // namespace zenslam
