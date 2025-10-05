#pragma once

#include <map>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include "keypoint.h"

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
         * @brief Create a grid detector with the specified cell size and underlying detector
         *
         * @param detector The feature detector to use within each grid cell
         * @param cell_size The size of each grid cell in pixels
         */
        grid_detector(const cv::Ptr<cv::Feature2D> &detector, const cv::Ptr<cv::Feature2D> &describer, cv::Size cell_size);

        void detect(cv::InputArray image_array, std::map<size_t, keypoint> &keypoints) const;

    private:
        cv::Ptr<cv::Feature2D> _detector; // Underlying detector
        cv::Ptr<cv::Feature2D> _describer;
        cv::Size               _cell_size; // Size of each grid cell
    };
} // namespace zenslam
