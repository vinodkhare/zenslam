#include "calibration.h"

#include <fstream>
#include <stdexcept>

#include <magic_enum/magic_enum.hpp>
#include <spdlog/spdlog.h>

#include <yaml-cpp/yaml.h>

#include "utils.h"

// read camera calibration from a kalibr yaml file
auto zenslam::calibration::parse(const std::filesystem::path &path, const std::string &camera_name) -> calibration
{
    if (!std::filesystem::exists(path))
    {
        throw std::runtime_error("Calibration file not found: " + path.string());
    }

    auto config = YAML::LoadFile(path.string());

    // Read cam0 (first camera) by default
    if (!config[camera_name])
    {
        throw std::runtime_error(camera_name + " not found in calibration file");
    }

    auto cam = config[camera_name];

    calibration calib;

    calib.camera_name = camera_name;

    // Parse resolution
    if (cam["resolution"] && cam["resolution"].size() == 2)
    {
        calib.resolution = cv::Size
        (
            cam["resolution"][0].as<int>(),
            cam["resolution"][1].as<int>()
        );
    }

    // Parse intrinsics [fx, fy, cx, cy]
    if (cam["intrinsics"] && cam["intrinsics"].size() == 4)
    {
        calib.focal_length = cv::Vec2d
        (
            cam["intrinsics"][0].as<double>(),
            cam["intrinsics"][1].as<double>()
        );

        calib.principal_point = cv::Vec2d
        (
            cam["intrinsics"][2].as<double>(),
            cam["intrinsics"][3].as<double>()
        );
    }

    // Parse distortion coefficients
    if (cam["distortion_coeffs"])
    {
        calib.distortion_coefficients.clear();
        for (const auto &coeff: cam["distortion_coeffs"])
        {
            calib.distortion_coefficients.push_back(coeff.as<double>());
        }
    }

    // Parse distortion model
    if (cam["distortion_model"])
    {
        const auto &optional   = magic_enum::enum_cast<enum distortion_model>(cam["distortion_model"].as<std::string>());
        calib.distortion_model = optional.has_value() ? optional.value() : distortion_model::radial_tangential;
    }

    return calib;
}

auto zenslam::calibration::print() const -> void
{
    SPDLOG_INFO("Camera name: {}", camera_name);
    SPDLOG_INFO("  Resolution: {}x{}", resolution.width, resolution.height);
    SPDLOG_INFO("  Focal length: [{}, {}]", focal_length[0], focal_length[1]);
    SPDLOG_INFO("  Principal point: [{}, {}]", principal_point[0], principal_point[1]);
    SPDLOG_INFO("  Distortion model: {}", magic_enum::enum_name(distortion_model));
    SPDLOG_INFO("  Distortion coefficients: [{}]", zenslam::utils::to_string(distortion_coefficients));
}
