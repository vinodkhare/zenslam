#include "calibration.h"

#include <spdlog/spdlog.h>

auto zenslam::calibration::parse
(
    const std::filesystem::path& camera_calib_path,
    const std::filesystem::path& imu_calib_path
) -> calibration
{
    calibration calib { };

    calib.cameras[0]            = camera_calibration::parse(camera_calib_path, "cam0");
    calib.cameras[1]            = camera_calibration::parse(camera_calib_path, "cam1");
    calib.camera_matrix[0]      = calib.cameras[0].camera_matrix();
    calib.camera_matrix[1]      = calib.cameras[1].camera_matrix();
    calib.fundamental_matrix[0] = calib.cameras[0].fundamental(calib.cameras[1]);
    calib.fundamental_matrix[1] = calib.cameras[1].fundamental(calib.cameras[0]);
    calib.projection_matrix[0]  = calib.cameras[0].projection();
    calib.projection_matrix[1]  = calib.cameras[1].projection();
    calib.imu                   = imu_calibration::parse(imu_calib_path);

    // Compute stereo rectification matrices
    const auto& relative_pose = calib.cameras[1].pose_in_cam0;
    const auto& R = relative_pose.rotation();
    const auto& t = relative_pose.translation();
    
    cv::Mat R1_mat, R2_mat, P1_mat, P2_mat, Q_mat;
    cv::stereoRectify(
        calib.camera_matrix[0], calib.cameras[0].distortion_coefficients,
        calib.camera_matrix[1], calib.cameras[1].distortion_coefficients,
        calib.cameras[0].resolution,
        R, t,
        R1_mat, R2_mat, P1_mat, P2_mat, Q_mat,
        cv::CALIB_ZERO_DISPARITY, -1, calib.cameras[0].resolution
    );
    
    // Convert cv::Mat to cv::Matx
    calib.R1 = cv::Matx33d(R1_mat);
    calib.R2 = cv::Matx33d(R2_mat);
    calib.P1 = cv::Matx34d(P1_mat);
    calib.P2 = cv::Matx34d(P2_mat);
    calib.Q  = cv::Matx44d(Q_mat);

    return calib;
}

void zenslam::calibration::print() const
{
    SPDLOG_INFO("");
    cameras[0].print();
    SPDLOG_INFO("");
    cameras[1].print();
}