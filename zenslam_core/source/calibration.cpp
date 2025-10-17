#include "calibration.h"

#include <spdlog/spdlog.h>

auto zenslam::calibration::parse
(
    const std::filesystem::path &camera_calib_path,
    const std::filesystem::path &imu_calib_path
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

    return calib;
}

void zenslam::calibration::print() const
{
    SPDLOG_INFO("");
    cameras[0].print();
    SPDLOG_INFO("");
    cameras[1].print();
}