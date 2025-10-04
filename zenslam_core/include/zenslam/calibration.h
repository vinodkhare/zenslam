#pragma once

#include <filesystem>

#include <opencv2/core/affine.hpp>
#include <opencv2/core/types.hpp>

namespace zenslam
{
    class calibration
    {
    public:
        enum class distortion_model
        {
            equidistant,
            radial_tangential
        };

        static calibration parse(const std::filesystem::path &path, const std::string &camera_name);

        std::string         camera_name             = { };
        cv::Size            resolution              = { };
        cv::Vec2d           focal_length            = { };
        cv::Vec2d           principal_point         = { };
        std::vector<double> distortion_coefficients = { };
        distortion_model    distortion_model        = { distortion_model::radial_tangential };
        cv::Affine3d        pose_in_cam0            = { cv::Affine3d::Identity() };

        [[nodiscard]] auto camera_matrix() const -> cv::Matx33d;
        [[nodiscard]] auto fundamental(const calibration& other) const -> cv::Matx33d;
        [[nodiscard]] auto projection() const -> cv::Matx34d;

        auto print() const -> void;
    };
}
