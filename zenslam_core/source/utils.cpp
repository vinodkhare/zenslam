#include "utils.h"

#include <ranges>

#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/video/tracking.hpp>

#include <spdlog/spdlog.h>


auto zenslam::utils::draw_keypoints(const mono_frame &frame) -> cv::Mat
{
    const auto &keypoints = utils::cast<cv::KeyPoint>(values(frame.keypoints));

    // DRAW_RICH_KEYPOINTS shows size and orientation
    cv::Mat keypoints_image { };

    cv::drawKeypoints
    (
        frame.undistorted,
        keypoints,
        keypoints_image,
        cv::Scalar(0, 255, 0),
        cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS
    );

    return keypoints_image;
}

auto zenslam::utils::draw_matches(const stereo_frame &frame) -> cv::Mat
{
    cv::Mat matches_image { };

    const auto &keypoints_l = values(frame.l.keypoints);
    const auto &keypoints_r = values(frame.r.keypoints);

    std::vector<cv::DMatch> matches { };

    for (auto query = 0; query < keypoints_l.size(); query++)
    {
        for (auto train = 0; train < keypoints_r.size(); train++)
        {
            if (keypoints_l[query].index == keypoints_r[train].index)
            {
                matches.emplace_back(query, train, 1.0);
            }
        }
    }

    cv::drawMatches
    (
        frame.l.undistorted,
        utils::cast<cv::KeyPoint>(keypoints_l),
        frame.r.undistorted,
        utils::cast<cv::KeyPoint>(keypoints_r),
        matches,
        matches_image,
        cv::Scalar(000, 255, 000),
        cv::Scalar(255, 000, 000),
        std::vector<char>(),
        cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS
    );

    return matches_image;
}

auto zenslam::utils::draw_matches(const mono_frame &frame_0, const mono_frame &frame_1) -> cv::Mat
{
    cv::Mat matches_image { };

    const auto &keypoints_0 = values(frame_0.keypoints);
    const auto &keypoints_1 = values(frame_1.keypoints);

    std::vector<cv::DMatch> matches { };

    for (auto query = 0; query < keypoints_0.size(); query++)
    {
        for (auto train = 0; train < keypoints_1.size(); train++)
        {
            if (keypoints_0[query].index == keypoints_1[train].index)
            {
                matches.emplace_back(query, train, 0);
            }
        }
    }

    cv::drawMatches
    (
        frame_0.undistorted,
        utils::cast<cv::KeyPoint>(keypoints_0),
        frame_1.undistorted,
        utils::cast<cv::KeyPoint>(keypoints_1),
        matches,
        matches_image,
        cv::Scalar(000, 255, 000),
        cv::Scalar(255, 000, 000),
        std::vector<char>(),
        cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS
    );

    return matches_image;
}

auto zenslam::utils::pyramid(const cv::Mat &image, const class options::slam &options) -> std::vector<cv::Mat>
{
    std::vector<cv::Mat> pyramid { };
    cv::buildOpticalFlowPyramid(image, pyramid, options.klt_window_size, options.klt_max_level);
    return pyramid;
}

auto zenslam::utils::skew(const cv::Vec3d &vector) -> cv::Matx33d
{
    auto skew = cv::Matx33d::zeros();

    skew(0, 1) = -vector[2];
    skew(1, 0) = vector[2];
    skew(0, 2) = vector[1];
    skew(2, 0) = -vector[1];
    skew(1, 2) = -vector[0];
    skew(2, 1) = vector[0];

    return skew;
}

auto zenslam::utils::to_keypoints(const std::vector<keypoint> &keypoints) -> std::vector<cv::KeyPoint>
{
    std::vector<cv::KeyPoint> cv_keypoints;
    cv_keypoints.reserve(keypoints.size());

    std::ranges::transform
    (
        keypoints,
        std::back_inserter(cv_keypoints),
        [](const auto &keypoint)
        {
            return keypoint;
        }
    );

    return cv_keypoints;
}

auto zenslam::utils::to_map(const std::vector<cv::DMatch> &matches) -> std::map<int, int>
{
    std::map<int, int> map;
    std::ranges::transform
    (
        matches,
        std::inserter(map, map.begin()),
        [](const auto &match)
        {
            return std::make_pair(match.queryIdx, match.trainIdx);
        }
    );
    return map;
}

auto zenslam::utils::to_points
(
    const std::vector<cv::KeyPoint> &keypoints0,
    const std::vector<cv::KeyPoint> &keypoints1,
    const std::vector<cv::DMatch> &  matches
) -> std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point2f>>
{
    std::vector<cv::Point2f> points0;
    points0.reserve(matches.size());

    std::vector<cv::Point2f> points1;
    points1.reserve(matches.size());

    for (const auto &match: matches)
    {
        points0.push_back(keypoints0[match.queryIdx].pt);
        points1.push_back(keypoints1[match.trainIdx].pt);
    }

    return std::make_tuple(points0, points1);
}

auto zenslam::utils::to_points(const std::vector<cv::KeyPoint> &keypoints) -> std::vector<cv::Point2f>
{
    std::vector<cv::Point2f> points;
    cv::KeyPoint::convert(keypoints, points);
    return points;
}

auto zenslam::utils::to_points(const std::map<size_t, keypoint> &keypoints) -> std::vector<cv::Point2f>
{
    std::vector<cv::Point2f> points;
    points.reserve(keypoints.size());

    for (const auto &value: keypoints | std::views::values)
    {
        points.emplace_back(value.pt);
    }

    return points;
}

