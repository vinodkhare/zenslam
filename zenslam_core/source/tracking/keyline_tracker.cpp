#include "zenslam/tracking/keyline_tracker.h"

#include <ranges>

#include <opencv2/features2d.hpp>

#include "zenslam/detection/detector.h"
#include "zenslam/tracking/tracking_utils.h"

namespace zenslam
{
    keyline_tracker::keyline_tracker(calibration calib, slam_options opts, frame::system& system) :
        _calibration(std::move(calib)),
        _tracking(opts.tracking),
        _detection(opts.detection),
        _triangulation(opts.triangulation),
        _detector(detector::create(opts.detection)),
        _system(system)
    {
    }

    std::vector<keyline> keyline_tracker::track_keylines(
        const std::vector<cv::Mat>& pyramid_0,
        const std::vector<cv::Mat>& pyramid_1,
        const map<keyline>&         keylines_map_0) const
    {
        slam_options options;
        options.tracking = _tracking;
        return utils::track_keylines(pyramid_0, pyramid_1, keylines_map_0, options);
    }

    auto keyline_tracker::assign_landmark_indices(
        std::vector<keyline>& keylines,
        const line3d_cloud&   lines3d,
        double                max_descriptor_distance) -> void
    {
        if (keylines.empty() || lines3d.empty())
        {
            return;
        }

        std::vector<size_t>  landmark_indices;
        std::vector<cv::Mat> landmark_descriptors;
        landmark_indices.reserve(lines3d.size());
        landmark_descriptors.reserve(lines3d.size());

        for (const auto& [idx, line3d] : lines3d)
        {
            if (line3d.descriptor.empty())
            {
                continue;
            }

            landmark_indices.emplace_back(idx);
            landmark_descriptors.emplace_back(line3d.descriptor);
        }

        if (landmark_descriptors.empty())
        {
            return;
        }

        std::vector<size_t>  keyline_rows;
        std::vector<cv::Mat> keyline_descriptors;
        keyline_rows.reserve(keylines.size());
        keyline_descriptors.reserve(keylines.size());

        for (size_t i = 0; i < keylines.size(); ++i)
        {
            if (keylines[i].descriptor.empty())
            {
                continue;
            }

            keyline_rows.emplace_back(i);
            keyline_descriptors.emplace_back(keylines[i].descriptor);
        }

        if (keyline_descriptors.empty())
        {
            return;
        }

        cv::Mat descriptors2d;
        cv::Mat descriptors3d;
        cv::vconcat(keyline_descriptors, descriptors2d);
        cv::vconcat(landmark_descriptors, descriptors3d);

        const auto is_binary = descriptors2d.depth() == CV_8U && descriptors3d.depth() == CV_8U;
        if (!is_binary)
        {
            if (descriptors2d.depth() != CV_32F)
            {
                descriptors2d.convertTo(descriptors2d, CV_32F);
            }
            if (descriptors3d.depth() != CV_32F)
            {
                descriptors3d.convertTo(descriptors3d, CV_32F);
            }
        }

        cv::BFMatcher           matcher(is_binary ? cv::NORM_HAMMING : cv::NORM_L2, false);
        std::vector<cv::DMatch> matches;
        matcher.match(descriptors2d, descriptors3d, matches);

        for (const auto& match : matches)
        {
            if (match.distance <= max_descriptor_distance)
            {
                auto& keyline = keylines[keyline_rows[match.queryIdx]];
                keyline.index = landmark_indices[match.trainIdx];
            }
        }
    }
} // namespace zenslam
