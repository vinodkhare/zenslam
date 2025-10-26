#include "zenslam/utils.h"

#include <ranges>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <spdlog/spdlog.h>


auto zenslam::utils::skew(const cv::Vec3d& vector) -> cv::Matx33d
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

auto zenslam::utils::to_keypoints(const std::vector<keypoint>& keypoints) -> std::vector<cv::KeyPoint>
{
    std::vector<cv::KeyPoint> cv_keypoints;
    cv_keypoints.reserve(keypoints.size());

    std::ranges::transform
    (
        keypoints,
        std::back_inserter(cv_keypoints),
        [](const auto& keypoint)
        {
            return keypoint;
        }
    );

    return cv_keypoints;
}

auto zenslam::utils::to_map(const std::vector<cv::DMatch>& matches) -> std::map<int, int>
{
    std::map<int, int> map;
    std::ranges::transform
    (
        matches,
        std::inserter(map, map.begin()),
        [](const auto& match)
        {
            return std::make_pair(match.queryIdx, match.trainIdx);
        }
    );
    return map;
}

auto zenslam::utils::to_points
(
    const std::vector<cv::KeyPoint>& keypoints0,
    const std::vector<cv::KeyPoint>& keypoints1,
    const std::vector<cv::DMatch>&   matches
) -> std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point2f>>
{
    std::vector<cv::Point2f> points0;
    points0.reserve(matches.size());

    std::vector<cv::Point2f> points1;
    points1.reserve(matches.size());

    for (const auto& match : matches)
    {
        points0.push_back(keypoints0[match.queryIdx].pt);
        points1.push_back(keypoints1[match.trainIdx].pt);
    }

    return std::make_tuple(points0, points1);
}

auto zenslam::utils::to_points(const std::vector<cv::KeyPoint>& keypoints) -> std::vector<cv::Point2f>
{
    std::vector<cv::Point2f> points;
    cv::KeyPoint::convert(keypoints, points);
    return points;
}

auto zenslam::utils::to_points(const std::vector<keypoint>& keypoints) -> std::vector<cv::Point2f>
{
    return keypoints | std::views::transform
    (
        [](const auto& keypoint)
        {
            return keypoint.pt;
        }
    ) | std::ranges::to<std::vector>();
}

auto zenslam::utils::to_points(const std::map<size_t, keypoint>& keypoints) -> std::vector<cv::Point2f>
{
    std::vector<cv::Point2f> points;
    points.reserve(keypoints.size());

    for (const auto& value : keypoints | std::views::values)
    {
        points.emplace_back(value.pt);
    }

    return points;
}

auto zenslam::utils::to_points(const std::vector<point3d>& points3d) -> std::vector<cv::Point3d>
{
    std::vector<cv::Point3d> points;
    points.reserve(points3d.size());

    for (const auto& point : points3d)
    {
        points.emplace_back(point);
    }

    return points;
}

auto zenslam::utils::matrix_to_euler(const cv::Matx33d& R) -> cv::Vec3d
{
    // Handle singularity when cos(pitch) = 0 (gimbal lock case)
    if (std::abs(R(2, 0)) >= 1.0 - 1e-8)
    {
        // Gimbal lock case
        constexpr auto yaw   = 0.0;                                   // Set yaw to zero as it's arbitrary in gimbal lock
        const auto     pitch = -M_PI_2 * std::copysign(1.0, R(2, 0)); // -pi/2 if R(2,0)=1, pi/2 if R(2,0)=-1
        const auto     roll  = yaw + std::atan2(-std::copysign(1.0, R(2, 0)) * R(0, 1), R(1, 1));

        return { roll, pitch, yaw };
    }

    // Normal case
    const auto roll  = std::atan2(R(2, 1), R(2, 2));
    const auto pitch = -std::asin(R(2, 0));
    const auto yaw   = std::atan2(R(1, 0), R(0, 0));

    return { roll, pitch, yaw };
}

auto zenslam::utils::rectify(const cv::Mat& image, const cv::Mat& map_x, const cv::Mat& map_y) -> cv::Mat
{
    cv::Mat rectified;
    cv::remap(image, rectified, map_x, map_y, cv::INTER_LINEAR);
    return rectified;
}