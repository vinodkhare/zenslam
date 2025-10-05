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
    class grid_detector final : public cv::Feature2D
    {
    public:
        /**
         * @brief Create a grid detector with the specified cell size and underlying detector
         *
         * @param detector The feature detector to use within each grid cell
         * @param cell_size The size of each grid cell in pixels
         */
        grid_detector(const cv::Ptr<Feature2D> &detector, cv::Size cell_size);

        /**
         * @brief Create a grid detector with the specified cell size and underlying detector
         *
         * @param detector The feature detector to use within each grid cell
         * @param cell_size The size of each grid cell in pixels
         * @return cv::Ptr<grid_detector> Smart pointer to the created detector
         */
        static cv::Ptr<grid_detector> create(const cv::Ptr<Feature2D> &detector, cv::Size cell_size);


        // Override methods from the cv::Feature2D base class

        /**
         * @brief Detect keypoints in an image using a grid-based approach
         *
         * Divides the image into grid cells and detects the strongest keypoint in each cell.
         *
         * @param image_array Input image
         * @param keypoints Detected keypoints
         * @param mask_array Optional mask specifying where to look for keypoints
         */
        void detect
        (
            cv::InputArray             image_array,
            std::vector<cv::KeyPoint> &keypoints,
            cv::InputArray             mask_array
        ) override;

        void detect
        (
            cv::InputArray              image_array,
            std::map<size_t, keypoint> &keypoints
        ) const;

        /**
         * @brief Get the algorithm descriptor name
         *
         * @return std::string Name of the algorithm
         */
        [[nodiscard]] std::string getDefaultName() const override;

    protected:
        // Implementing the required method from cv::Feature2D
        void detectAndCompute
        (
            cv::InputArray             image,
            cv::InputArray             mask,
            std::vector<cv::KeyPoint> &keypoints,
            cv::OutputArray            descriptors,
            bool                       useProvidedKeypoints
        ) override;

    private:
        cv::Ptr<Feature2D> _detector;  // Underlying detector
        cv::Size           _cell_size; // Size of each grid cell
    };
} // namespace zenslam
