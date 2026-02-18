#include "zenslam/detection/detector.h"

#include <future>
#include <ranges>

#include <gsl/gsl>

#include <opencv2/imgproc.hpp> // For cv::Rect
#include <opencv2/xfeatures2d.hpp>

namespace zenslam
{
    auto detector::create(const slam_options& options) -> detector
    {
        auto feature_detector  = cv::Ptr<cv::Feature2D>();
        auto feature_describer = cv::Ptr<cv::Feature2D>();

        switch (options.feature)
        {
        case feature_type::FAST:
            feature_detector = cv::FastFeatureDetector::create(options.fast_threshold);
            break;
        case feature_type::ORB:
            feature_detector = cv::ORB::create(500, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, options.fast_threshold);
            break;
        case feature_type::SIFT:
            feature_detector = cv::SIFT::create();
            break;
        }

        switch (options.descriptor)
        {
        case descriptor_type::ORB:
            feature_describer = cv::ORB::create();
            break;
        case descriptor_type::SIFT:
            feature_describer = cv::SIFT::create();
            break;
        case descriptor_type::FREAK:
            feature_describer = cv::xfeatures2d::FREAK::create();
            break;
        }

        detector detector { };

        detector._detector      = feature_detector;
        detector._describer     = feature_describer;
        detector._cell_size     = options.cell_size;
        detector._line_detector = cv::line_descriptor::LSDDetector::createLSDDetector();

        return detector;
    }

    cv::Size operator/(const cv::Size& lhs, const cv::Size& rhs)
    {
        return { lhs.width / rhs.width, lhs.height / rhs.height };
    }

    auto detector::detect_keypoints
    (
        const cv::Mat&       image,
        const map<keypoint>& keypoints_existing
    ) const -> std::vector<keypoint>
    {
        // Calculate grid dimensions
        const auto& grid_size = cv::Size(image.cols, image.rows) / _cell_size;

        // Create a vector of vectors of size grid_size.rows * grid_size.cols
        std::vector occupied(grid_size.width, std::vector(grid_size.height, false));

        // loop over all existing keypoints and update occupancy
        for (const auto& keypoint : keypoints_existing | std::views::values)
        {
            // Calculate which grid cell this keypoint falls into
            const auto& grid_x = static_cast<int>(keypoint.pt.x) / _cell_size.width;
            const auto& grid_y = static_cast<int>(keypoint.pt.y) / _cell_size.height;

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

    auto detector::detect_keypoints_par
    (
        const cv::Mat&       image,
        const map<keypoint>& keypoints_existing
    ) const -> std::vector<keypoint>
    {
        // Calculate grid dimensions
        const auto& grid_size = cv::Size(image.cols, image.rows) / _cell_size;

        // Create a vector of vectors of size grid_size.rows * grid_size.cols
        std::vector occupied(grid_size.width, std::vector(grid_size.height, false));

        // loop over all existing keypoints and update occupancy
        for (const auto& keypoint : keypoints_existing | std::views::values)
        {
            // Calculate which grid cell this keypoint falls into
            const auto& grid_x = static_cast<int>(keypoint.pt.x) / _cell_size.width;
            const auto& grid_y = static_cast<int>(keypoint.pt.y) / _cell_size.height;

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

        // Collect all unoccupied cells
        std::vector<std::pair<int, int>> cells_to_process;
        for (auto y = 0; y < grid_size.height; ++y)
        {
            for (auto x = 0; x < grid_size.width; ++x)
            {
                if (!occupied[x][y])
                {
                    cells_to_process.emplace_back(x, y);
                }
            }
        }

        // Process cells in parallel
        std::vector<std::future<std::optional<cv::KeyPoint>>> futures;
        futures.reserve(cells_to_process.size());

        for (const auto& [x, y] : cells_to_process)
        {
            futures.push_back
            (
                std::async
                (
                    std::launch::async,
                    [this, &image, x, y]()
                    {
                        // Calculate cell boundaries, making sure not to exceed image dimensions
                        const cv::Rect cell_rect
                        (
                            x * _cell_size.width,
                            y * _cell_size.height,
                            std::min(_cell_size.width, image.cols - x * _cell_size.width),
                            std::min(_cell_size.height, image.rows - y * _cell_size.height)
                        );

                        // Extract the region of interest for this cell
                        const auto cell_image = image(cell_rect);

                        // Detect features in this cell
                        std::vector<cv::KeyPoint> cell_keypoints;
                        _detector->detect(cell_image, cell_keypoints, cv::noArray());

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

                            return std::optional { cell_keypoints[index] };
                        }

                        return std::optional<cv::KeyPoint> { };
                    }
                )
            );
        }

        // Collect results from all futures
        std::vector<cv::KeyPoint> keypoints_cv;
        keypoints_cv.reserve(futures.size());

        for (auto& future : futures)
        {
            auto result = future.get();
            if (result.has_value())
            {
                keypoints_cv.push_back(result.value());
            }
        }

        // Refine keypoint positions
        auto points_cv = keypoints_cv | std::views::transform
        (
            [](const auto& keypoint)
            {
                return keypoint.pt;
            }
        ) | std::ranges::to<std::vector>();

        if (!points_cv.empty())
        {
            cv::cornerSubPix
            (
                image,
                points_cv,
                cv::Size(5, 5),
                cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01)
            );
        }

        for (auto i = 0; i < points_cv.size(); ++i)
        {
            keypoints_cv[i].pt = points_cv[i];
        }

        // Compute descriptors
        cv::Mat  descriptors;
        cv::UMat image_umat;
        image.copyTo(image_umat);
        _describer->compute(image_umat, keypoints_cv, descriptors);

        std::vector<keypoint> keypoints = { };

        for (auto i = 0; i < descriptors.rows; ++i)
        {
            keypoints.emplace_back(keypoints_cv[i], keypoint::index_next, descriptors.row(i));

            keypoint::index_next++;
        }

        return keypoints;
    }

    auto detector::detect_keylines(const cv::Mat& image, const map<keyline>& keylines_map, const int mask_margin) const -> std::vector<keyline>
    {
        // Create a mask to block detection in areas where keylines already exist
        cv::Mat mask = cv::Mat::ones(image.size(), CV_8U) * 255;

        // For each existing keyline, mask out a thick line along the line segment
        for (const auto& existing_keyline : keylines_map | std::views::values)
        {
            // Get start and end points of the line segment
            const cv::Point start
            (
                static_cast<int>(existing_keyline.startPointX),
                static_cast<int>(existing_keyline.startPointY)
            );
            const cv::Point end
            (
                static_cast<int>(existing_keyline.endPointX),
                static_cast<int>(existing_keyline.endPointY)
            );

            // Draw a thick line on the mask (thickness = 2 * mask_margin)
            // This masks out a band of width 2*mask_margin centered on the line
            cv::line(mask, start, end, cv::Scalar(0), 2 * mask_margin, cv::LINE_8);
        }

        // Detect keylines with the mask
        std::vector<cv::line_descriptor::KeyLine> keylines_cv { };
        _line_detector->detect(image, keylines_cv, 2.0f, 1, mask);

        // Compute descriptors for detected keylines
        const auto& bd = cv::line_descriptor::BinaryDescriptor::createBinaryDescriptor();
        cv::Mat     descriptors { };
        if (!keylines_cv.empty())
        {
            bd->compute(image, keylines_cv, descriptors);
        }

        std::vector<keyline> keylines { };
        for (size_t i = 0; i < keylines_cv.size(); ++i)
        {
            if (keylines_cv[i].startPointY > keylines_cv[i].endPointY)
            {
                std::swap(keylines_cv[i].startPointX, keylines_cv[i].endPointX);
                std::swap(keylines_cv[i].startPointY, keylines_cv[i].endPointY);
            }

            keylines.emplace_back(keylines_cv[i], keyline::index_next);
            if (!descriptors.empty() && i < static_cast<size_t>(descriptors.rows))
                keylines.back().descriptor = descriptors.row(static_cast<int>(i)).clone();
            keyline::index_next++;
        }

        return keylines;
    }
} // namespace zenslam