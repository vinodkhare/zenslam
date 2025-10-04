#include "utils.h"

#include <chrono>

#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>

auto zenslam::utils::draw_keypoints(const mono_frame &frame) -> cv::Mat
{
    cv::Mat keypoints_image { };

    // DRAW_RICH_KEYPOINTS shows size and orientation
    cv::drawKeypoints
    (
        frame.image,
        frame.keypoints,
        keypoints_image,
        cv::Scalar(0, 255, 0),
        cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS
    );

    return keypoints_image;
}

auto zenslam::utils::draw_matches(const stereo_frame &frame) -> cv::Mat
{
    cv::Mat matches_image { };

    cv::drawMatches
    (
        frame.l.undistorted,
        frame.l.keypoints,
        frame.r.undistorted,
        frame.r.keypoints,
        frame.matches,
        matches_image,
        cv::Scalar(0, 255, 0),
        cv::Scalar(0, 255, 0),
        std::vector<char>(),
        cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS
    );

    return matches_image;
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
    std::vector<cv::DMatch>  filtered;
    std::vector<cv::Point2f> pts0, pts1;

    // Extract matching points
    for (const auto &m: matches)
    {
        pts0.push_back(keypoints0[m.queryIdx].pt);
        pts1.push_back(keypoints1[m.trainIdx].pt);
    }

    if (pts0.size() == pts1.size() && !pts0.empty())
    {
        std::vector<cv::Vec3f> epilines0, epilines1;
        cv::computeCorrespondEpilines(pts0, 1, fundamental, epilines1);
        cv::computeCorrespondEpilines(pts1, 2, fundamental, epilines0);

        for (size_t i = 0; i < matches.size(); ++i)
        {
            const auto &pt0   = pts0[i];
            const auto &pt1   = pts1[i];
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

auto zenslam::utils::keypoints_to_points(const std::vector<cv::KeyPoint> &keypoints) -> std::vector<cv::Point2f>
{
    std::vector<cv::Point2f> points;
    cv::KeyPoint::convert(keypoints, points);
    return points;
}


