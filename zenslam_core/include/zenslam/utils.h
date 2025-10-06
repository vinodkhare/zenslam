#pragma once

#include <functional>
#include <map>
#include <numeric>
#include <ranges>
#include <string>

#include <spdlog/common.h>
#include <spdlog/fmt/ostr.h> // must be included

#include "calibration.h"
#include "stereo_folder_reader.h"
#include "stereo_frame.h"

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
    template <typename T_OUT, typename T_IN>
    auto cast(const std::vector<T_IN> &values) -> std::vector<T_OUT>
    {
        return values | std::views::transform([](const T_IN &value) { return static_cast<T_OUT>(value); }) | std::ranges::to<std::vector>();
    }

    template <typename T_KEY, typename T_VALUE>
    auto invert(const std::map<T_KEY, T_VALUE> &map) -> std::map<T_VALUE, T_KEY>
    {
        return map | std::views::transform([](const auto &pair) { return std::make_pair(pair.second, pair.first); }) | std::ranges::to<std::map>();
    }

    // Generic template function to join any range of elements into a string
    template <std::ranges::input_range Range, typename Projection = std::identity>
        requires std::invocable<Projection, std::ranges::range_reference_t<Range>>
    auto join_to_string(const Range &range, const std::string &delimiter, Projection projection = { }) -> std::string
    {
        if (std::ranges::empty(range))
        {
            return "";
        }

        auto transformed = range | std::ranges::views::transform(projection);
        auto it          = transformed.begin();

        return std::accumulate
        (
            std::next(it),
            transformed.end(),
            std::string(std::invoke(projection, *std::ranges::begin(range))),
            [&delimiter](const std::string &a, const auto &b)
            {
                return a + delimiter + std::string(b);
            }
        );
    }

    template <typename T_KEY, typename T_VALUE>
    auto values(const std::map<T_KEY, T_VALUE> &map) -> std::vector<T_VALUE>
    {
        std::vector<T_VALUE> values;
        values.reserve(map.size());

        for (const auto &value: map)
        {
            values.push_back(value.second);
        }

        return values;
    }

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

    auto to_string(const std::vector<std::string> &strings, const std::string &delimiter = ", ") -> std::string;
    auto to_string(const std::array<std::string_view, 8> &strings, const std::string &delimiter = ", ") -> std::string;
    auto to_string(const std::vector<double> &values, const std::string &delimiter = ", ") -> std::string;
    auto to_string_epoch(double epoch_seconds) -> std::string;

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
