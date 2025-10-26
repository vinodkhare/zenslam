#include "zenslam/matcher.h"

#include <ranges>

#include <opencv2/calib3d.hpp>

#include "zenslam/utils_slam.h"

namespace zenslam
{
    matcher::matcher(const class options::slam& opts, const bool is_binary)
        : _options(opts)
    {
        _matcher = utils::create_matcher(_options, is_binary);
    }

    auto matcher::match_keypoints(const map<keypoint>& keypoints_0, const map<keypoint>& keypoints_1) const -> std::vector<cv::DMatch>
    {
        cv::Mat                 descriptors_l { };
        cv::Mat                 descriptors_r { };
        std::vector<keypoint>   unmatched_l { };
        std::vector<keypoint>   unmatched_r { };
        std::vector<cv::DMatch> matches_new { };

        for (const auto& keypoint_l : keypoints_0 | std::views::values)
        {
            if (keypoints_1.contains(keypoint_l.index))
                continue;

            unmatched_l.emplace_back(keypoint_l);

            if (descriptors_l.empty())
            {
                descriptors_l = keypoint_l.descriptor;
            }
            else
            {
                cv::vconcat(descriptors_l, keypoint_l.descriptor, descriptors_l);
            }
        }

        for (const auto& keypoint_r : keypoints_1 | std::views::values)
        {
            if (keypoints_0.contains(keypoint_r.index))
                continue;

            unmatched_r.emplace_back(keypoint_r);

            if (descriptors_r.empty())
            {
                descriptors_r = keypoint_r.descriptor;
            }
            else
            {
                cv::vconcat(descriptors_r, keypoint_r.descriptor, descriptors_r);
            }
        }

        if (descriptors_l.empty() || descriptors_r.empty())
            return matches_new;

        std::vector<cv::DMatch> matches { };
        const bool              use_ratio_test { _options.matcher == matcher_type::KNN || _options.matcher == matcher_type::FLANN };

        if (use_ratio_test)
        {
            // kNN matching with ratio test (for KNN and FLANN modes)
            std::vector<std::vector<cv::DMatch>> knn_matches;
            _matcher->knnMatch(descriptors_l, descriptors_r, knn_matches, 2);

            // Apply Lowe's ratio test
            for (const auto& knn : knn_matches)
            {
                if (knn.size() == 2 &&
                    knn[0].distance < _options.matcher_ratio * knn[1].distance)
                {
                    matches.push_back(knn[0]);
                }
            }
        }
        else
        {
            // Brute-force matching with cross-check (1-NN)
            _matcher->match(descriptors_l, descriptors_r, matches);
        }

        // Estimate fundamental matrix using RANSAC and filter matches
        if (matches.size() >= 8)
        {
            auto points_l = matches |
                std::views::transform
                (
                    [&unmatched_l](const auto& match)
                    {
                        return unmatched_l[match.queryIdx].pt;
                    }
                ) |
                std::ranges::to<std::vector>();

            auto points_r = matches |
                std::views::transform
                (
                    [&unmatched_r](const auto& match)
                    {
                        return unmatched_r[match.trainIdx].pt;
                    }
                ) |
                std::ranges::to<std::vector>();

            std::vector<uchar> inlier_mask;
            cv::Mat            fundamental =
                cv::findFundamentalMat
                (
                    points_l,
                    points_r,
                    cv::FM_RANSAC,
                    _options.epipolar_threshold,
                    0.99,
                    inlier_mask
                );

            // Keep only inliers
            std::vector<cv::DMatch> filtered_matches;
            for (size_t i = 0; i < matches.size(); ++i)
            {
                if (inlier_mask[i])
                {
                    filtered_matches.push_back(matches[i]);
                }
            }

            matches = filtered_matches;
        }

        for (const auto& match : matches)
        {
            const auto& keypoint_l = unmatched_l[match.queryIdx];
            const auto& keypoint_r = unmatched_r[match.trainIdx];

            matches_new.emplace_back
            (
                keypoint_l.index,
                keypoint_r.index,
                match.distance
            );
        }

        return matches_new;
    }
}
