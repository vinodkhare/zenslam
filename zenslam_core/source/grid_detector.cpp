#include "grid_detector.h"

#include <gsl/gsl>

#include <opencv2/imgproc.hpp> // For cv::Rect

namespace zenslam
{
    grid_detector::grid_detector(const cv::Ptr<Feature2D> &detector, const cv::Size cellSize) :
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
        const auto image = image_array.getMat();

        // Calculate grid dimensions
        const auto &grid_size = cv::Size(image.cols, image.rows) / _cell_size;

        // Create a vector of vectors of size grid_size.rows * grid_size.cols
        std::vector occupied(grid_size.width, std::vector(grid_size.height, false));

        // loop over all existing keypoints and update occupancy
        for (const auto &keypoint: keypoints)
        {
            // Calculate which grid cell this keypoint falls into
            const auto &grid_x = static_cast<int>(keypoint.pt.x) / _cell_size.width;
            const auto &grid_y = static_cast<int>(keypoint.pt.y) / _cell_size.height;

            // If the keypoint falls within grid bounds, mark the cell as occupied
            if
            (
                grid_x >= 0 && grid_x < grid_size.width &&
                grid_y >= 0 && grid_y < grid_size.height
            )
            {
                occupied[grid_x][grid_y] = true;
            }
        }

        // For each cell in the grid
        for (auto y = 0; y < grid_size.height; ++y)
        {
            for (auto x = 0; x < grid_size.width; ++x)
            {
                // If the cell is occupied then continue
                if (occupied[x][y]) continue;

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

                // Detect features in this cell
                std::vector<cv::KeyPoint> cell_keypoints;
                _detector->detect(cell_image, cell_keypoints, cv::noArray());

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
        const bool                 useProvidedKeypoints
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
