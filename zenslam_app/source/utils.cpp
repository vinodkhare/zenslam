#include "utils.h"

#include <chrono>

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
        frame.l.image,
        frame.l.keypoints,
        frame.r.image,
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

std::string zenslam::utils::epoch_double_to_string(const double epoch_seconds)
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


