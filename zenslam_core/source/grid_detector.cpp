#include "grid_detector.h"

#include <future>
#include <ranges>

#include <gsl/gsl>

#include <opencv2/imgproc.hpp> // For cv::Rect

namespace zenslam
{
    grid_detector::grid_detector
    (
        const cv::Ptr<cv::Feature2D> &detector,
        const cv::Ptr<cv::Feature2D> &describer,
        const cv::Size                cell_size
    ) :
        _detector(detector),
        _describer { describer },
        _cell_size(cell_size)
    {
        // Validate input
        CV_Assert(detector);
        CV_Assert(cell_size.width > 0 && cell_size.height > 0);
    }

    cv::Size operator/(const cv::Size &lhs, const cv::Size &rhs)
    {
        return { lhs.width / rhs.width, lhs.height / rhs.height };
    }

    void grid_detector::detect(cv::InputArray image_array, std::map<size_t, keypoint> &keypoints_map) const
    {
        const auto image = image_array.getMat();

        // Calculate grid dimensions
        const auto &grid_size = cv::Size(image.cols, image.rows) / _cell_size;

        // Create a vector of vectors of size grid_size.rows * grid_size.cols
        std::vector occupied(grid_size.width, std::vector(grid_size.height, false));

        // loop over all existing keypoints and update occupancy
        for (const auto &keypoint: keypoints_map | std::views::values)
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

        std::vector<size_t> indices { };

        // For each cell in the grid
        for (auto y = 0; y < grid_size.height; ++y)
        {
            for (auto x = 0; x < grid_size.width; ++x)
            {
                // If the cell is occupied, then continue
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

                // if (cell_keypoints.empty())
                // {
                //     _describer->detect(cell_image, cell_keypoints, cv::noArray());
                // }

                // If any keypoints were found in this cell
                if (!cell_keypoints.empty())
                {
                    // Find the keypoint with the highest response in this cell
                    const auto &index = std::distance
                    (
                        cell_keypoints.begin(),
                        std::ranges::max_element
                        (
                            cell_keypoints,
                            [](const cv::KeyPoint &a, const cv::KeyPoint &b)
                            {
                                return a.response < b.response;
                            }
                        )
                    );

                    // Adjust keypoint coordinates to be relative to the original image
                    cell_keypoints[index].pt.x += gsl::narrow<float, int>(cell_rect.x);
                    cell_keypoints[index].pt.y += gsl::narrow<float, int>(cell_rect.y);

                    // Add the best keypoint from this cell
                    keypoint keypoint { cell_keypoints[index], keypoint::index_next++ };

                    keypoints_map.emplace(keypoint.index, keypoint);
                    indices.emplace_back(keypoint.index);
                }
            }
        }

        const auto &keypoints = keypoints_map | std::views::values | std::ranges::to<std::vector>();

        auto keypoints_cv = keypoints | std::views::transform
                            (
                                [](const auto &keypoint) -> cv::KeyPoint
                                {
                                    return keypoint;
                                }
                            ) | std::ranges::to<std::vector>();

        cv::Mat descriptors;
        _describer->compute(image, keypoints_cv, descriptors);

        if (keypoints_cv.size() != keypoints.size())
        {
            throw std::runtime_error("Keypoint count mismatch");
        }

        for (auto i = 0; i < descriptors.rows; ++i)
        {
            keypoints_map[keypoints[i].index].descriptor = descriptors.row(i);
        }
    }

    void grid_detector::detect_par(cv::InputArray image_array, std::map<size_t, keypoint> &keypoints_map) const
    {
        const auto  image     = image_array.getMat();
        const auto &grid_size = cv::Size(image.cols, image.rows) / _cell_size;

        // Create occupancy grid
        std::vector occupied(grid_size.width, std::vector(grid_size.height, false));

        // Mark occupied cells
        for (const auto &keypoint: keypoints_map | std::views::values)
        {
            const auto &grid_x = static_cast<int>(keypoint.pt.x) / _cell_size.width;
            const auto &grid_y = static_cast<int>(keypoint.pt.y) / _cell_size.height;

            if (grid_x >= 0 && grid_x < grid_size.width &&
                grid_y >= 0 && grid_y < grid_size.height)
            {
                occupied[grid_x][grid_y] = true;
            }
        }

        // Create a vector of cell coordinates to process
        struct cell_info
        {
            cv::Rect rect;
            int      x, y;
        };
        std::vector<cell_info> cells_to_process { };

        // Fill cells to process
        for (auto y = 0; y < grid_size.height; ++y)
        {
            for (auto x = 0; x < grid_size.width; ++x)
            {
                if (!occupied[x][y])
                {
                    cv::Rect cell_rect
                    (
                        x * _cell_size.width,
                        y * _cell_size.height,
                        std::min(_cell_size.width, image.cols - x * _cell_size.width),
                        std::min(_cell_size.height, image.rows - y * _cell_size.height)
                    );
                    cells_to_process.push_back({ cell_rect, x, y });
                }
            }
        }

        // Process cells in parallel using async
        struct cell_result
        {
            keypoint keypoint;
            bool     valid;
        };

        // Launch async tasks for each cell
        std::vector<std::future<cell_result>> futures;
        futures.reserve(cells_to_process.size());

        for (const auto &cell: cells_to_process)
        {
            futures.push_back
            (
                std::async
                (
                    std::launch::async,
                    [this, &image, cell]() -> cell_result
                    {
                        const auto                cell_image = image(cell.rect);
                        std::vector<cv::KeyPoint> cell_keypoints;

                        _detector->detect(cell_image, cell_keypoints, cv::noArray());

                        if (cell_keypoints.empty())
                        {
                            return { keypoint { }, false };
                        }

                        // Find best keypoint
                        const auto &max_it = std::ranges::max_element
                        (
                            cell_keypoints,
                            [](const cv::KeyPoint &a, const cv::KeyPoint &b)
                            {
                                return a.response < b.response;
                            }
                        );

                        // Adjust coordinates
                        auto best_kp = *max_it;
                        best_kp.pt.x += gsl::narrow<float, int>(cell.rect.x);
                        best_kp.pt.y += gsl::narrow<float, int>(cell.rect.y);

                        // Create keypoint with new index
                        const keypoint kp { best_kp, keypoint::index_next++ };
                        return { kp, true };
                    }
                )
            );
        }

        // Collect results from futures
        std::vector<keypoint> new_keypoints;
        new_keypoints.reserve(futures.size());

        for (auto &future: futures)
        {
            auto [keypoint, valid] = future.get();

            if (valid)
            {
                new_keypoints.push_back(keypoint);
                keypoints_map.emplace(keypoint.index, keypoint);
            }
        }

        // If we found new keypoints, compute their descriptors
        if (!new_keypoints.empty())
        {
            // Convert to OpenCV keypoints
            std::vector<cv::KeyPoint> keypoints_cv;
            keypoints_cv.reserve(new_keypoints.size());

            for (const auto &kp: new_keypoints)
            {
                keypoints_cv.push_back(kp);
            }

            // Compute descriptors
            cv::Mat descriptors;
            _describer->compute(image, keypoints_cv, descriptors);

            if (keypoints_cv.size() != new_keypoints.size())
            {
                throw std::runtime_error("Keypoint count mismatch in parallel detection");
            }

            // Update descriptors
            for (size_t i = 0; i < descriptors.rows; ++i)
            {
                keypoints_map[new_keypoints[i].index].descriptor = descriptors.row(i);
            }
        }
    }
} // namespace zenslam
