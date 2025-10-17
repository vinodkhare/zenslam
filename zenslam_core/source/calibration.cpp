#include "calibration.h"

#include <spdlog/spdlog.h>

auto zenslam::calibration::parse
(
    const std::filesystem::path &camera_calib_path,
    const std::filesystem::path &imu_calib_path
) -> calibration
{
    calibration calib { };

    calib.camera[0]             = camera_calibration::parse(camera_calib_path, "cam0");
    calib.camera[1]             = camera_calibration::parse(camera_calib_path, "cam1");
    calib.camera_matrix[0]      = calib.camera[0].camera_matrix();
    calib.camera_matrix[1]      = calib.camera[1].camera_matrix();
    calib.fundamental_matrix[0] = calib.camera[0].fundamental(calib.camera[1]);
    calib.fundamental_matrix[1] = calib.camera[1].fundamental(calib.camera[0]);
    calib.projection_matrix[0]  = calib.camera[0].projection();
    calib.projection_matrix[1]  = calib.camera[1].projection();
    calib.imu                   = imu_calibration::parse(imu_calib_path);

    return calib;
}

void zenslam::calibration::print() const
{
    SPDLOG_INFO("");
    camera[0].print();
    SPDLOG_INFO("");
    camera[1].print();
}
