#include "utils.h"

#include <chrono>

#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>

#include <spdlog/spdlog.h>

auto zenslam::utils::draw_keypoints(const mono_frame &frame) -> cv::Mat
{
    const auto &keypoints = utils::cast<cv::KeyPoint>(values(frame.keypoints_));

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

    const auto &keypoints_l = values(frame.l.keypoints_);
    const auto &keypoints_r = values(frame.r.keypoints_);

    std::vector<cv::DMatch> matches { };

    for (auto query = 0; query < keypoints_l.size(); query++)
    {
        for (auto train = 0; train < keypoints_r.size(); train++)
        {
            if (keypoints_l[query].index == keypoints_r[train].index)
            {
                matches.emplace_back(cv::DMatch(query, train, 1.0));
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

    const auto &keypoints_0 = values(frame_0.keypoints_);
    const auto &keypoints_1 = values(frame_1.keypoints_);

    std::vector<cv::DMatch> matches { };

    for (auto query = 0; query < keypoints_0.size(); query++)
    {
        for (auto train = 0; train < keypoints_1.size(); train++)
        {
            if (keypoints_0[query].index == keypoints_1[train].index)
            {
                matches.emplace_back(cv::DMatch(query, train, 0));
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
) -> auto
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

auto zenslam::utils::to_string(const std::vector<std::string> &strings, const std::string &delimiter) -> std::string
{
    return join_to_string(strings, delimiter, std::identity { });
}

auto zenslam::utils::to_string(const std::array<std::string_view, 8> &strings, const std::string &delimiter) -> std::string
{
    return join_to_string(strings, delimiter, std::identity { });
}

auto zenslam::utils::to_string(const std::vector<double> &values, const std::string &delimiter) -> std::string
{
    return join_to_string
    (
        values,
        delimiter,
        [](const double value)
        {
            return std::to_string(value);
        }
    );
}

std::string zenslam::utils::to_string_epoch(const double epoch_seconds)
{
    // Split into integral seconds and fractional milliseconds
    const auto &seconds      = std::chrono::floor<std::chrono::seconds>(std::chrono::duration<double>(epoch_seconds));
    const auto &milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>
            (std::chrono::duration<double>(epoch_seconds) - seconds);

    // sys_time with milliseconds precision
    auto time_point = std::chrono::sys_seconds(seconds) + milliseconds;

    // Format: YYYY-MM-DD HH:MM:SS.mmm UTC
    return std::format("{:%F %T} UTC", time_point);
}

auto zenslam::utils::filter
(
    const std::vector<cv::KeyPoint> &keypoints0,
    const std::vector<cv::KeyPoint> &keypoints1,
    const std::vector<cv::DMatch> &  matches,
    const cv::Matx33d &              fundamental,
    const double                     epipolar_threshold
) -> std::vector<cv::DMatch>
{
    std::vector<cv::DMatch> filtered { };
    filtered.reserve(matches.size());

    if
    (
        auto [points0, points1] = to_points(keypoints0, keypoints1, matches);
        points0.size() == points1.size() && !points0.empty()
    )
    {
        std::vector<cv::Vec3f> epilines0, epilines1;
        cv::computeCorrespondEpilines(points0, 1, fundamental, epilines1);
        cv::computeCorrespondEpilines(points1, 2, fundamental, epilines0);

        for (size_t i = 0; i < matches.size(); ++i)
        {
            const auto &pt0   = points0[i];
            const auto &pt1   = points1[i];
            const auto &line1 = epilines1[i];
            const auto &line0 = epilines0[i];

            const double err0 = std::abs(line0[0] * pt0.x + line0[1] * pt0.y + line0[2]) /
                                std::sqrt(line0[0] * line0[0] + line0[1] * line0[1]);

            const double err1 = std::abs(line1[0] * pt1.x + line1[1] * pt1.y + line1[2]) /
                                std::sqrt(line1[0] * line1[0] + line1[1] * line1[1]);

            if (err0 < epipolar_threshold && err1 < epipolar_threshold)
            {
                filtered.push_back(matches[i]);
            }
        }
    }
    else
    {
        filtered = matches;
    }

    return filtered;
}

auto zenslam::utils::match
(
    const std::map<size_t, keypoint> &keypoints_0,
    std::map<size_t, keypoint> &      keypoints_1,
    const cv::Matx33d &               fundamental,
    double                            epipolar_threshold
) -> void
{
    cv::Mat descriptors_l;
    cv::Mat descriptors_r;

    for (const auto &keypoint_L: keypoints_0 | std::views::values)
    {
        if (descriptors_l.empty())
        {
            descriptors_l = keypoint_L.descriptor;
        }
        else
        {
            cv::vconcat(descriptors_l, keypoint_L.descriptor, descriptors_l);
        }
    }

    for (const auto &keypoint_R: keypoints_1 | std::views::values)
    {
        if (descriptors_r.empty())
        {
            descriptors_r = keypoint_R.descriptor;
        }
        else
        {
            cv::vconcat(descriptors_r, keypoint_R.descriptor, descriptors_r);
        }
    }

    const cv::BFMatcher     matcher { cv::NORM_L2, true };
    std::vector<cv::DMatch> matches;
    matcher.match(descriptors_l, descriptors_r, matches);

    SPDLOG_INFO("matches count: {}", matches.size());

    matches = filter
    (
        utils::cast<cv::KeyPoint>(values(keypoints_0)),
        utils::cast<cv::KeyPoint>(values(keypoints_1)),
        matches,
        fundamental,
        epipolar_threshold
    );

    SPDLOG_INFO("matches filtered count: {}", matches.size());

    const auto &keypoints_L = keypoints_0 | std::ranges::views::values | std::ranges::to<std::vector>();
    const auto &keypoints_R = keypoints_1 | std::ranges::views::values | std::ranges::to<std::vector>();

    for (const auto &match: matches)
    {
        const auto &keypoint_L = keypoints_L[match.queryIdx];
        auto        keypoint_R = keypoints_R[match.trainIdx];

        keypoints_1.erase(keypoint_R.index);

        keypoint_R.index = keypoint_L.index;

        keypoints_1.emplace(keypoint_R.index, keypoint_R);
    }
}

auto zenslam::utils::triangulate
(
    const std::vector<cv::KeyPoint> &keypoints0,
    const std::vector<cv::KeyPoint> &keypoints1,
    const std::vector<cv::DMatch> &  matches,
    const cv::Matx34d &              projection0,
    const cv::Matx34d &              projection1
) -> std::vector<cv::Point3d>
{
    auto [points0, points1] = to_points(keypoints0, keypoints1, matches);

    cv::Mat points4d;
    cv::triangulatePoints(projection0, projection1, points0, points1, points4d);

    std::vector<cv::Point3d> points3d;
    points3d.reserve(points4d.cols);

    for (auto c = 0; c < points4d.cols; ++c)
    {
        if
        (
            cv::Vec4d col = points4d.col(c);

            std::abs(col[3]) > 1e-9
        )
        {
            points3d.emplace_back
            (
                col[0] / col[3],
                col[1] / col[3],
                col[2] / col[3]
            );
        }
    }

    return points3d;
}

auto zenslam::utils::triangulate
(
    const stereo_frame &            frame,
    const cv::Matx34d &             projection_l,
    const cv::Matx34d &             projection_r,
    std::map<unsigned long, point> &points
) -> void
{
    std::vector<cv::Point2f> points_l { };
    std::vector<cv::Point2f> points_r { };
    std::vector<size_t>      indices { };

    for (const auto &[index, keypoint_L]: frame.l.keypoints_)
    {
        if (frame.r.keypoints_.contains(index))
        {
            const auto &keypoint_R = frame.r.keypoints_.at(index);

            points_l.emplace_back(keypoint_L.pt);
            points_r.emplace_back(keypoint_R.pt);

            indices.emplace_back(index);
        }
    }

    cv::Mat points4d;
    cv::triangulatePoints(projection_l, projection_r, points_l, points_r, points4d);

    std::vector<cv::Point3d> points3d;
    points3d.reserve(points4d.cols);

    for (auto c = 0; c < points4d.cols; ++c)
    {
        cv::Vec4d point4d = points4d.col(c);

        if (std::abs(point4d[3]) > 1e-9)
        {
            points[indices[c]] = point { { point4d[0] / point4d[3], point4d[1] / point4d[3], point4d[2] / point4d[3] }, indices[c] };
        }
    }
}

auto zenslam::utils::undistort(const cv::Mat &image, const calibration &calibration) -> cv::Mat
{
    cv::Mat undistorted { };

    switch (calibration.distortion_model)
    {
        case calibration::distortion_model::radial_tangential:
            cv::undistort
            (
                image,
                calibration.camera_matrix(),
                calibration.distortion_coefficients,
                undistorted,
                calibration.camera_matrix()
            );
            break;

        case calibration::distortion_model::equidistant:
            cv::fisheye::undistortImage
            (
                image,
                undistorted,
                calibration.camera_matrix(),
                calibration.distortion_coefficients,
                calibration.camera_matrix()
            );
            break;
    }

    return undistorted;
}


