#pragma once

#include <map>
#include <string>

#include <spdlog/common.h>
#include <spdlog/fmt/ostr.h>

#include "options.h"
#include "stereo_frame.h"
#include "utils_std.h"

// Pretty formatter for cv::Affine3d for spdlog/fmt
template <>
struct fmt::formatter<cv::Affine3d> : formatter<std::string>
{
    template <typename FormatContext>
    auto format(const cv::Affine3d &value, FormatContext &context) const
    {
        const auto &R = value.rotation();
        const auto &t = value.translation();

        auto s = fmt::format
        (
            "[\n  R = \n  [\n    {:+.4f} {:+.4f} {:+.4f}\n    {:+.4f} {:+.4f} {:+.4f}\n    {:+.4f} {:+.4f} {:+.4f}\n  ]\n  t = [{:+.4f} {:+.4f} {:+.4f}]\n]",
            R(0, 0),
            R(0, 1),
            R(0, 2),
            R(1, 0),
            R(1, 1),
            R(1, 2),
            R(2, 0),
            R(2, 1),
            R(2, 2),
            t[0],
            t[1],
            t[2]
        );

        return formatter<std::string>::format(s, context);
    }
};

namespace zenslam::utils
{
    inline std::string version = "0.0.1";

    inline std::map<std::string, spdlog::level::level_enum> log_levels_from_string =
    {
        { "trace", spdlog::level::trace },
        { "debug", spdlog::level::debug },
        { "info", spdlog::level::info },
        { "warn", spdlog::level::warn },
        { "error", spdlog::level::err },
        { "critical", spdlog::level::critical },
        { "off", spdlog::level::off }
    };

    inline auto log_levels_to_string = invert(log_levels_from_string);

    auto skew(const cv::Vec3d &vector) -> cv::Matx33d;
    auto to_keypoints(const std::vector<keypoint> &keypoints) -> std::vector<cv::KeyPoint>;
    auto to_map(const std::vector<cv::DMatch> &matches) -> std::map<int, int>;

    auto to_points
    (
        const std::vector<cv::KeyPoint> &keypoints0,
        const std::vector<cv::KeyPoint> &keypoints1,
        const std::vector<cv::DMatch> &  matches
    ) -> std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point2f>>;

    auto to_points(const std::vector<cv::KeyPoint> &keypoints) -> std::vector<cv::Point2f>;
    auto to_points(const std::map<size_t, keypoint> &keypoints) -> std::vector<cv::Point2f>;

    /**
     * Convert a 3x3 rotation matrix to Euler angles using ZYX (yaw-pitch-roll) convention
     * @param R Input rotation matrix
     * @return cv::Vec3d containing Euler angles in radians in the order [roll, pitch, yaw]
     *         roll  (x): Rotation around X axis [-pi,   pi]
     *         pitch (y): Rotation around Y axis [-pi/2, pi/2]
     *         yaw   (z): Rotation around Z axis [-pi,   pi]
     */
    auto matrix_to_euler(const cv::Matx33d &R) -> cv::Vec3d;
}
