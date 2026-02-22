#include "zenslam/detection/keyline_detector.h"

#include <future>
#include <ranges>

#include <gsl/gsl>

#include <opencv2/imgproc.hpp> // For cv::Rect
#include <opencv2/xfeatures2d.hpp>

#include "zenslam/all_options.h"
#include "zenslam/utils/utils_opencv.h"

namespace zenslam
{
    keyline_detector::keyline_detector(const detection_options& options) :
        _options(options)
    {
    }

    auto keyline_detector::detect_keylines(const cv::Mat& image, const map<keyline>& keylines_map, const int mask_margin) const -> std::vector<keyline>
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
        _detector->detect(image, keylines_cv, 2.0f, 1, mask);

        // Compute descriptors for detected keylines
        cv::Mat descriptors { };
        if (!keylines_cv.empty())
        {
            _describer->compute(image, keylines_cv, descriptors);
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
            {
                keylines.back().descriptor = descriptors.row(gsl::narrow_cast<int>(i)).clone();
            }

            keyline::index_next++;
        }

        return keylines | std::views::filter
        (
            [this](const auto& keyline)
            {
                const auto dx     = keyline.endPointX - keyline.startPointX;
                const auto dy     = keyline.endPointY - keyline.startPointY;
                const auto length = std::sqrt(dx * dx + dy * dy);

                return length <= _options.keyline_max_length;
            }
        ) | std::ranges::to<std::vector>();
    }
} // namespace zenslam
