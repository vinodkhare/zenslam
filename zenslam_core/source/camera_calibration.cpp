#include "zenslam/camera_calibration.h"


#include <magic_enum/magic_enum.hpp>

#include <opencv2/calib3d.hpp>

#include <spdlog/spdlog.h>

#include <yaml-cpp/yaml.h>

#include "zenslam/utils.h"
#include "zenslam/formatters.h"

// read camera calibration from a kalibr yaml file
auto zenslam::camera_calibration::parse(const std::filesystem::path& path, const std::string& camera_name) -> camera_calibration
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

    camera_calibration calib;

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
        for (const auto& coeff : cam["distortion_coeffs"])
        {
            calib.distortion_coefficients.push_back(coeff.as<double>());
        }
    }

    // Parse distortion model
    if (cam["distortion_model"])
    {
        const auto& optional = magic_enum::enum_cast <
        enum distortion_model
        >
        (cam["distortion_model"].as<std::string>());
        calib.distortion_model = optional.has_value() ? optional.value() : distortion_model::radial_tangential;
    }

    auto node = cam["T_cn_cnm1"];

    // Parse pose
    if (cam["T_cn_cnm1"] && cam["T_cn_cnm1"].size() == 4)
    {
        cv::Matx44d T;
        for (auto i = 0; i < 16; ++i)
        {
            T(i / 4, i % 4) = cam["T_cn_cnm1"][i / 4][i % 4].as<double>();
        }
        calib.pose_in_cam0 = cv::Affine3d(T).inv();
    }

    // Parse pose
    if (cam["T_cam_imu"] && cam["T_cam_imu"].size() == 4)
    {
        cv::Matx44d T;
        for (auto i = 0; i < 16; ++i)
        {
            T(i / 4, i % 4) = cam["T_cam_imu"][i / 4][i % 4].as<double>();
        }
        calib.pose_in_imu0 = cv::Affine3d(T);
    }

    return calib;
}

auto zenslam::camera_calibration::camera_matrix() const -> cv::Matx33d
{
    return
    {
        focal_length[0],
        0,
        principal_point[0],
        0,
        focal_length[1],
        principal_point[1],
        0,
        0,
        1
    };
}

auto zenslam::camera_calibration::print() const -> void
{
    SPDLOG_INFO("Camera name: {}", camera_name);
    SPDLOG_INFO("  Resolution: {}x{}", resolution.width, resolution.height);
    SPDLOG_INFO("  Focal length: [{}, {}]", focal_length[0], focal_length[1]);
    SPDLOG_INFO("  Principal point: [{}, {}]", principal_point[0], principal_point[1]);
    SPDLOG_INFO("  Distortion model: {}", magic_enum::enum_name(distortion_model));
    SPDLOG_INFO("  Distortion coefficients: [{}]", zenslam::utils::to_string(distortion_coefficients));
    SPDLOG_INFO("  Pose in cam0: {}", pose_in_cam0);
}

auto zenslam::camera_calibration::fundamental(const camera_calibration& other) const -> cv::Matx33d
{
    // Get camera matrices
    const auto K1 = this->camera_matrix();
    const auto K2 = other.camera_matrix();

    // Get the relative pose between cameras
    const auto& relative_pose = other.pose_in_cam0.inv() * this->pose_in_cam0;
    const auto& t             = relative_pose.translation();
    const auto& R             = relative_pose.rotation();
    const auto& E             = utils::skew(t) * R; // Compute essential matrix E = [t]x * R

    // Compute fundamental matrix F = K2^-T * E * K1^-1
    return K2.inv().t() * E * K1.inv();
}

auto zenslam::camera_calibration::projection() const -> cv::Matx34d
{
    // Projection matrix P = K * [R | t]
    const auto K = camera_matrix();

    // Take a 3x4 minor (top 3 rows, 4 cols) from the 4x4 affine matrix => [R|t]
    const auto Rt = pose_in_cam0.inv().matrix.get_minor < 3, 
    4 > (0, 0);

    // P = K * [R|t] =>
    return K * Rt; // Matx (3x3) * (3x4) => (3x4)
}

auto zenslam::camera_calibration::projection(const cv::Affine3d& pose_of_cam0_in_world) const -> cv::Matx34d
{
    // Projection matrix P = K * [R | t]
    const auto K = camera_matrix();

    const auto& pose_of_this_in_world = pose_of_cam0_in_world * pose_in_cam0;

    // Take a 3x4 minor (top 3 rows, 4 cols) from the 4x4 affine matrix => [R|t]
    const auto Rt = pose_of_this_in_world.inv().matrix.get_minor < 3, 
    4 > (0, 0);

    // P = K * [R|t] =>
    return K * Rt; // Matx (3x3) * (3x4) => (3x4)
}