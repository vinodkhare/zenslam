#pragma once

#include <map>
#include <string>

#include <spdlog/common.h>
#include <spdlog/fmt/ostr.h> // must be included

#include "calibration.h"
#include "stereo_folder_reader.h"
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

    auto draw_keypoints(const mono_frame &frame) -> cv::Mat;
    auto draw_matches(const stereo_frame &frame) -> cv::Mat;
    auto draw_matches(const mono_frame &frame_0, const mono_frame &frame_1) -> cv::Mat;
    auto skew(const cv::Vec3d &vector) -> cv::Matx33d;
    auto to_keypoints(const std::vector<keypoint> &keypoints) -> std::vector<cv::KeyPoint>;
    auto to_map(const std::vector<cv::DMatch> &matches) -> std::map<int, int>;

    auto to_points
    (
        const std::vector<cv::KeyPoint> &keypoints0,
        const std::vector<cv::KeyPoint> &keypoints1,
        const std::vector<cv::DMatch> &  matches
    ) -> auto;

    auto to_points(const std::vector<cv::KeyPoint> &keypoints) -> std::vector<cv::Point2f>;
    auto to_points(const std::map<size_t, keypoint> &keypoints) -> std::vector<cv::Point2f>;

    // filters matches using the epipolar crterion given the fundamental matrix
    auto filter
    (
        const std::vector<cv::KeyPoint> &keypoints0,
        const std::vector<cv::KeyPoint> &keypoints1,
        const std::vector<cv::DMatch> &  matches,
        const cv::Matx33d &              fundamental,
        double                           epipolar_threshold
    ) -> std::vector<cv::DMatch>;

    auto match
    (
        const std::map<size_t, keypoint> &keypoints_0,
        std::map<size_t, keypoint> &      keypoints_1,
        const cv::Matx33d &               fundamental,
        double                            epipolar_threshold
    ) -> void;

    auto triangulate
    (
        stereo_frame &            frame,
        const cv::Matx34d &             projection_l,
        const cv::Matx34d &             projection_r,
        std::map<unsigned long, point> &points
    ) -> void;

    auto undistort(const cv::Mat &image, const zenslam::calibration &calibration) -> cv::Mat;

    auto umeyama(const std::vector<cv::Point3d> &src, const std::vector<cv::Point3d> &dst, cv::Matx33d &R, cv::Vec3d &t) -> void;
}
