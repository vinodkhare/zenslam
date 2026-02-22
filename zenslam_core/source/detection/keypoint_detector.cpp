#include "zenslam/detection/keypoint_detector.h"

#include <gsl/narrow>

#include <opencv2/xfeatures2d.hpp>

#include "zenslam/utils/utils_opencv.h"

zenslam::keypoint_detector::keypoint_detector(const detection_options& options) :
    _options(options)
{
    switch (options.feature_detector)
    {
    case feature_type::FAST:
        _detector = cv::FastFeatureDetector::create(options.fast_threshold);
        break;
    case feature_type::ORB:
        _detector = cv::ORB::create(500, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, options.fast_threshold);
        break;
    case feature_type::SIFT:
        _detector = cv::SIFT::create();
        break;
    }

    switch (options.descriptor)
    {
    case descriptor_type::ORB:
        _describer = cv::ORB::create();
        break;
    case descriptor_type::SIFT:
        _describer = cv::SIFT::create();
        break;
    case descriptor_type::FREAK:
        _describer = cv::xfeatures2d::FREAK::create();
        break;
    }
}

std::vector<zenslam::keypoint> zenslam::keypoint_detector::detect_keypoints(const cv::Mat& image, const map<keypoint>& keypoints_existing) const
{
    // Calculate grid dimensions
        const auto& grid_size = cv::Size(image.cols, image.rows) / _options.cell_size;

        // Create a vector of vectors of size grid_size.rows * grid_size.cols
        std::vector occupied(grid_size.width, std::vector(grid_size.height, false));

        // loop over all existing keypoints and update occupancy
        for (const auto& keypoint : keypoints_existing | std::views::values)
        {
            // Calculate which grid cell this keypoint falls into
            const auto grid_x = gsl::narrow_cast<int>(keypoint.pt.x) / _options.cell_size.width;
            const auto grid_y = gsl::narrow_cast<int>(keypoint.pt.y) / _options.cell_size.height;

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

        std::vector<cv::KeyPoint> keypoints_cv { };

        // For each cell in the grid
        for (auto y = 0; y < grid_size.height; ++y)
        {
            for (auto x = 0; x < grid_size.width; ++x)
            {
                // If the cell is occupied, then continue
                if (occupied[x][y])
                    continue;

                // Calculate cell boundaries, making sure not to exceed image dimensions
                cv::Rect cell_rect
                (
                    x * _options.cell_size.width,
                    y * _options.cell_size.height,
                    std::min(_options.cell_size.width, image.cols - x * _options.cell_size.width),
                    std::min(_options.cell_size.height, image.rows - y * _options.cell_size.height)
                );

                // Extract the region of interest for this cell
                auto cell_image = image(cell_rect);

                // Detect features in this cell
                std::vector<cv::KeyPoint> cell_keypoints;
                _detector->detect(cell_image, cell_keypoints, cv::noArray());

                if (cell_keypoints.empty())
                {
                    _describer->detect(cell_image, cell_keypoints, cv::noArray());
                }

                // If any keypoints were found in this cell
                if (!cell_keypoints.empty())
                {
                    // Find the keypoint with the highest response in this cell
                    const auto& index = std::distance
                    (
                        cell_keypoints.begin(),
                        std::ranges::max_element
                        (
                            cell_keypoints,
                            [](const cv::KeyPoint& a, const cv::KeyPoint& b)
                            {
                                return a.response < b.response;
                            }
                        )
                    );

                    // Adjust keypoint coordinates to be relative to the original image
                    cell_keypoints[index].pt.x += gsl::narrow<float, int>(cell_rect.x);
                    cell_keypoints[index].pt.y += gsl::narrow<float, int>(cell_rect.y);

                    // Add the best keypoint from this cell
                    keypoints_cv.emplace_back(cell_keypoints[index]);
                }
            }
        }

        const auto points_cv = keypoints_cv | std::views::transform
        (
            [](const auto& keypoint)
            {
                return keypoint.pt;
            }
        ) | std::ranges::to<std::vector>();

        for (auto i = 0; i < points_cv.size(); ++i)
        {
            keypoints_cv[i].pt = points_cv[i];
        }

        cv::Mat descriptors;
        _describer->compute(image, keypoints_cv, descriptors);

        std::vector<keypoint> keypoints = { };

        for (auto i = 0; i < descriptors.rows; ++i)
        {
            keypoints.emplace_back(keypoints_cv[i], keypoint::index_next, descriptors.row(i));

            keypoint::index_next++;
        }

        return keypoints;
}
