#pragma once

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/line_descriptor/descriptor.hpp>

#include "options.h"
#include "types/keyline.h"
#include "types/keypoint.h"
#include "types/map.h"

namespace zenslam
{
    /**
     * @brief Feature detector that divides an image into grid cells and detects one feature per cell
     *
     * This detector wraps any OpenCV feature detector and applies it to each grid cell
     * independently, returning the strongest keypoint from each cell. This ensures
     * a more uniform distribution of keypoints across the image.
     */
    class grid_detector final
    {
    public:
        /**
         * @brief Create a grid detector based on the provided SLAM options
         *
         * @param options The SLAM options containing the desired feature detector type and cell size
         * @return A configured grid_detector instance
         */
        static auto create(const class options::slam& options) -> grid_detector;

        /**
         * @brief Detect keypoints in the image using grid-based detection
         *
         * @param image The input image in which to detect keypoints
         * @param keypoints_existing A map to store detected keypoints with their indices
         * @return A vector of detected keypoints
         */
        [[nodiscard]] std::vector<keypoint> detect_keypoints(const cv::Mat& image, const map<keypoint>& keypoints_existing) const;

        /**
         * @brief Detect keypoints in the image using parallel grid-based detection
         *
         * This is a parallel version of detect_keypoints that processes grid cells concurrently
         * using std::async. It provides better performance on multi-core systems when detecting
         * features across many grid cells.
         *
         * @param image The input image in which to detect keypoints
         * @param keypoints_existing A map to store detected keypoints with their indices
         * @return A vector of detected keypoints
         */
        [[nodiscard]] std::vector<keypoint> detect_keypoints_par(const cv::Mat& image, const map<keypoint>& keypoints_existing) const;

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
        cv::Ptr<cv::Feature2D>                    _detector      = { }; // Underlying detector
        cv::Ptr<cv::Feature2D>                    _describer     = { };
        cv::Ptr<cv::line_descriptor::LSDDetector> _line_detector = { }; // Line feature detector
        cv::Size                                  _cell_size     = { }; // Size of each grid cell
    };
} // namespace zenslam
