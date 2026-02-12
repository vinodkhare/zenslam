#pragma once

#include <filesystem>

#include "zenslam/utils.h"

#include <opencv2/core/types.hpp>

#include <spdlog/spdlog.h>

#include <numbers>

// Pretty formatter for cv::Affine3d for spdlog/fmt
template <>
struct fmt::formatter<cv::Affine3d> : formatter<std::string>
{
    template <typename FormatContext>
    auto format(const cv::Affine3d& value, FormatContext& context) const
    {
        const auto& R      = value.rotation();
        const auto& t      = value.translation();
        const auto& angles = zenslam::utils::matrix_to_euler(R) * (180.0 / std::numbers::pi);

        return formatter<std::string>::format(
            fmt::format("{{ x: {{ {:+.4f}, {:+.4f}, {:+.4f} }}, 𝜭: {{ {:+.4f}, {:+.4f}, {:+.4f} }} }}", t[0], t[1], t[2], angles[0], angles[1], angles[2]),
            context);
    }
};

// Pretty formatter for cv::Vec3d for spdlog/fmt
template <>
struct fmt::formatter<cv::Vec3d> : formatter<std::string>
{
    template <typename FormatContext>
    auto format(const cv::Vec3d& value, FormatContext& context) const
    {
        return formatter<std::string>::format(fmt::format("[ {:+.4f}, {:+.4f}, {:+.4f} ]", value[0], value[1], value[2]), context);
    }
};

/**
 * SPDLOG formatter for std::filesystem::path
 */
template <>
struct fmt::formatter<std::filesystem::path> : formatter<std::string>
{
    template <typename FormatContext>
    auto format(const std::filesystem::path& value, FormatContext& context) const
    {  
        return formatter<std::string>::format(value.string(), context);
    }
};