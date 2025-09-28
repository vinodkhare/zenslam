#include "grid_detector.h"

#include <gsl/gsl>

#include <opencv2/imgproc.hpp> // For cv::Rect

namespace zenslam
{
    grid_detector::grid_detector(const cv::Ptr<Feature2D> &detector, cv::Size cellSize) :
        _detector(detector),
        _cell_size(cellSize)
    {
        // Validate input
        CV_Assert(detector);
        CV_Assert(cellSize.width > 0 && cellSize.height > 0);
    }

    cv::Ptr<grid_detector> grid_detector::create(const cv::Ptr<Feature2D> &detector, const cv::Size cell_size)
    {
        return cv::makePtr<grid_detector>(detector, cell_size);
    }

    cv::Size operator/(const cv::Size &lhs, const cv::Size &rhs)
    {
        return { lhs.width / rhs.width, lhs.height / rhs.height };
    }

    void grid_detector::detect
    (
        cv::InputArray             image_array,
        std::vector<cv::KeyPoint> &keypoints,
        cv::InputArray             mask_array
    )
    {
        keypoints.clear();

        const auto image = image_array.getMat();
        const auto mask  = mask_array.getMat();

        // Calculate grid dimensions
        const auto &grid_size = cv::Size(image.cols, image.rows) / _cell_size;

        // For each cell in the grid
        for (auto y = 0; y < grid_size.height; ++y)
        {
            for (auto x = 0; x < grid_size.width; ++x)
            {
                // Calculate cell boundaries, making sure not to exceed image dimensions
                cv::Rect cell_rect
                (
                    x * _cell_size.width,
                    y * _cell_size.height,
                    std::min(_cell_size.width, image.cols - x * _cell_size.width),
                    std::min(_cell_size.height, image.rows - y * _cell_size.height)
                );

                // Extract the region of interest for this cell
                auto cell_image = image(cell_rect);

                cv::Mat cell_mask;

                // If a mask is provided, extract the corresponding region
                if (!mask.empty())
                {
                    cell_mask = mask(cell_rect);
                }

                // Detect features in this cell
                std::vector<cv::KeyPoint> cell_keypoints;
                _detector->detect(cell_image, cell_keypoints, cell_mask);

                // If any keypoints were found in this cell
                if (!cell_keypoints.empty())
                {
                    // Find the keypoint with the highest response in this cell
                    auto best_keypoint = std::ranges::max_element
                    (
                        cell_keypoints,

                        [](const cv::KeyPoint &a, const cv::KeyPoint &b)
                        {
                            return a.response < b.response;
                        }
                    );

                    // Adjust keypoint coordinates to be relative to the original image
                    auto adjusted_keypoint = *best_keypoint;
                    adjusted_keypoint.pt.x += gsl::narrow<float, int>(cell_rect.x);
                    adjusted_keypoint.pt.y += gsl::narrow<float, int>(cell_rect.y);

                    // Add the best keypoint from this cell
                    keypoints.push_back(adjusted_keypoint);
                }
            }
        }
    }

    std::string grid_detector::getDefaultName() const
    {
        return "GridDetector";
    }

    void grid_detector::detectAndCompute
    (
        cv::InputArray             image,
        cv::InputArray             mask,
        std::vector<cv::KeyPoint> &keypoints,
        cv::OutputArray            descriptors,
        bool                       useProvidedKeypoints
    )
    {
        // If keypoints are not provided, detect them using our grid method
        if (!useProvidedKeypoints)
        {
            detect(image, keypoints, mask);
        }

        // If descriptors are requested, compute them using the underlying detector
        if (descriptors.needed())
        {
            _detector->compute(image, keypoints, descriptors);
        }
    }
} // namespace zenslam
