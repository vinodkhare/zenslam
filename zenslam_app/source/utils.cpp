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
            cv::undistort(image, calibration.camera_matrix(), calibration.distortion_coefficients, undistorted, calibration.camera_matrix());
            break;

        case calibration::distortion_model::equidistant:
            cv::fisheye::undistortImage(image, undistorted, calibration.camera_matrix(), calibration.distortion_coefficients, calibration.camera_matrix());
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


