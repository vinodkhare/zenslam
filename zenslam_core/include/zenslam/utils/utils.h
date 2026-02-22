#pragma once


#include <map>

#include <string>

#include <spdlog/common.h>
#include <spdlog/fmt/ostr.h>

#include "zenslam/types/keypoint.h"
#include "zenslam/types/point3d.h"
#include "zenslam/utils/utils_std.h"


namespace zenslam::utils
{
    inline constexpr std::string_view version = "0.0.1";

    inline std::map<std::string, spdlog::level::level_enum> log_levels_from_string = {
        { "trace", spdlog::level::trace },
        { "debug", spdlog::level::debug },
        { "info", spdlog::level::info },
        { "warn", spdlog::level::warn },
        { "error", spdlog::level::err },
        { "critical", spdlog::level::critical },
        { "off", spdlog::level::off }
    };

    inline auto log_levels_to_string = invert(log_levels_from_string);

    auto skew(const cv::Vec3d& vector) -> cv::Matx33d;
    auto to_keypoints(const std::vector<keypoint>& keypoints) -> std::vector<cv::KeyPoint>;
    auto to_map(const std::vector<cv::DMatch>& matches) -> std::map<int, int>;

    auto to_points
    (
        const std::vector<cv::KeyPoint>& keypoints0,
        const std::vector<cv::KeyPoint>& keypoints1,
        const std::vector<cv::DMatch>&   matches
    )
        -> std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point2f>>;

    auto to_points(const std::vector<cv::KeyPoint>& keypoints) -> std::vector<cv::Point2f>;
    auto to_points(const std::vector<keypoint>& keypoints) -> std::vector<cv::Point2f>;
    auto to_points(const std::map<size_t, keypoint>& keypoints) -> std::vector<cv::Point2f>;

    /**
     * Convert a vector of zenslam::point3d to a vector of cv::Point3d
     * @param points3d Input vector of zenslam::point3d
     * @return Vector of cv::Point3d
     */
    auto to_points(const std::vector<point3d>& points3d) -> std::vector<cv::Point3d>;

    /**
     * Convert a 3x3 rotation matrix to Euler angles using ZYX (yaw-pitch-roll) convention
     * @param R Input rotation matrix
     * @return cv::Vec3d containing Euler angles in radians in the order [roll, pitch, yaw]
     *         roll  (x): Rotation around X axis [-pi,   pi]
     *         pitch (y): Rotation around Y axis [-pi/2, pi/2]
     *         yaw   (z): Rotation around Z axis [-pi,   pi]
     */
    auto matrix_to_euler(const cv::Matx33d& R) -> cv::Vec3d;

    template <typename T>
    auto vecnorm(const std::vector<cv::Point_<T>>& vec) -> std::vector<T>
    {
        std::vector<T> vecnorm { };
        for (const auto& v : vec) { vecnorm.emplace_back(cv::norm(v)); }
        return vecnorm;
    }

    /**
     * Apply stereo rectification to an image using pre-computed maps
     * @param image Input image to rectify
     * @param map_x Pre-computed x rectification map
     * @param map_y Pre-computed y rectification map
     * @return Rectified image
     */
    auto rectify(const cv::Mat& image, const cv::Mat& map_x, const cv::Mat& map_y) -> cv::Mat;
} // namespace zenslam::utils

