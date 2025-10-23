#include "calibration.h"

#include <opencv2/calib3d.hpp>

#include <spdlog/spdlog.h>

#include "utils_opencv.h"

auto zenslam::calibration::parse
(
    const std::filesystem::path& camera_calib_path,
    const std::filesystem::path& imu_calib_path,
    const bool                   stereo_rectify
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

    auto rotation_0 = cv::Matx33d::eye();
    auto rotation_1 = cv::Matx33d::eye();
    auto Q_mat      = cv::Matx44d { };
    if (stereo_rectify)
    {
        // Compute stereo rectification matrices
        const auto& rotation    = calib.cameras[1].pose_in_cam0.inv().rotation();
        const auto& translation = calib.cameras[1].pose_in_cam0.inv().translation();

        if (calib.cameras[0].distortion_model == camera_calibration::distortion_model::radial_tangential)
        {
            cv::stereoRectify
            (
                calib.camera_matrix[0],
                calib.cameras[0].distortion_coefficients,
                calib.camera_matrix[1],
                calib.cameras[1].distortion_coefficients,
                calib.cameras[0].resolution,
                rotation,
                translation,
                rotation_0,
                rotation_1,
                calib.projection_matrix[0],
                calib.projection_matrix[1],
                Q_mat,
                cv::CALIB_ZERO_DISPARITY,
                -1,
                calib.cameras[0].resolution
            );

            cv::initUndistortRectifyMap
            (
                calib.camera_matrix[0],
                calib.cameras[0].distortion_coefficients,
                rotation_0,
                calib.projection_matrix[0],
                calib.cameras[0].resolution,
                CV_32FC1,
                calib.map_x[0],
                calib.map_y[0]
            );

            cv::initUndistortRectifyMap
            (
                calib.camera_matrix[1],
                calib.cameras[1].distortion_coefficients,
                rotation_1,
                calib.projection_matrix[1],
                calib.cameras[1].resolution,
                CV_32FC1,
                calib.map_x[1],
                calib.map_y[1]
            );
        }
        else if (calib.cameras[0].distortion_model == camera_calibration::distortion_model::equidistant)
        {
            cv::fisheye::stereoRectify
            (
                calib.camera_matrix[0],
                calib.cameras[0].distortion_coefficients,
                calib.camera_matrix[1],
                calib.cameras[1].distortion_coefficients,
                calib.cameras[0].resolution,
                rotation,
                translation,
                rotation_0,
                rotation_1,
                calib.projection_matrix[0],
                calib.projection_matrix[1],
                Q_mat,
                cv::CALIB_ZERO_DISPARITY,
                calib.cameras[0].resolution,
                0.0
            );

            cv::fisheye::initUndistortRectifyMap
            (
                calib.camera_matrix[0],
                calib.cameras[0].distortion_coefficients,
                rotation_0,
                calib.projection_matrix[0],
                calib.cameras[0].resolution,
                CV_32FC1,
                calib.map_x[0],
                calib.map_y[0]
            );

            cv::fisheye::initUndistortRectifyMap
            (
                calib.camera_matrix[1],
                calib.cameras[1].distortion_coefficients,
                rotation_1,
                calib.projection_matrix[1],
                calib.cameras[1].resolution,
                CV_32FC1,
                calib.map_x[1],
                calib.map_y[1]
            );
        }

        const auto result_0 = utils::projection_decompose(calib.projection_matrix[0]);
        const auto result_1 = utils::projection_decompose(calib.projection_matrix[1]);

        calib.camera_matrix[0]           = std::get<0>(result_0);
        calib.camera_matrix[1]           = std::get<0>(result_1);
        calib.cameras[0].focal_length    = { calib.camera_matrix[0](0, 0), calib.camera_matrix[0](1, 1) };
        calib.cameras[0].principal_point = { calib.camera_matrix[0](0, 2), calib.camera_matrix[0](1, 2) };
        calib.cameras[1].focal_length    = { calib.camera_matrix[1](0, 0), calib.camera_matrix[1](1, 1) };
        calib.cameras[1].principal_point = { calib.camera_matrix[1](0, 2), calib.camera_matrix[1](1, 2) };
        calib.cameras[0].pose_in_cam0    = cv::Affine3d(std::get<1>(result_0), std::get<2>(result_0)).inv();
        calib.cameras[1].pose_in_cam0    = cv::Affine3d(std::get<1>(result_1), std::get<2>(result_1)).inv();
        calib.fundamental_matrix[0]      = calib.cameras[0].fundamental(calib.cameras[1]);
        calib.fundamental_matrix[1]      = calib.cameras[1].fundamental(calib.cameras[0]);
    }
    else
    {
        // Pre-compute rectification maps for both cameras
        if (calib.cameras[0].distortion_model == camera_calibration::distortion_model::radial_tangential)
        {
            const auto camera_matrix_new_0 = cv::getOptimalNewCameraMatrix
                    (calib.camera_matrix[0], calib.cameras[0].distortion_coefficients, calib.cameras[0].resolution, 0);
            const auto camera_matrix_new_1 = cv::getOptimalNewCameraMatrix
                    (calib.camera_matrix[1], calib.cameras[1].distortion_coefficients, calib.cameras[1].resolution, 0);

            cv::initUndistortRectifyMap
            (
                calib.camera_matrix[0],
                calib.cameras[0].distortion_coefficients,
                rotation_0,
                camera_matrix_new_0,
                calib.cameras[0].resolution,
                CV_32FC1,
                calib.map_x[0],
                calib.map_y[0]
            );

            cv::initUndistortRectifyMap
            (
                calib.camera_matrix[1],
                calib.cameras[1].distortion_coefficients,
                rotation_1,
                camera_matrix_new_1,
                calib.cameras[1].resolution,
                CV_32FC1,
                calib.map_x[1],
                calib.map_y[1]
            );

            calib.camera_matrix[0] = camera_matrix_new_0;
            calib.camera_matrix[1] = camera_matrix_new_1;

            calib.cameras[0].focal_length    = { calib.camera_matrix[0](0, 0), calib.camera_matrix[0](1, 1) };
            calib.cameras[0].principal_point = { calib.camera_matrix[0](0, 2), calib.camera_matrix[0](1, 2) };
            calib.cameras[1].focal_length    = { calib.camera_matrix[1](0, 0), calib.camera_matrix[1](1, 1) };
            calib.cameras[1].principal_point = { calib.camera_matrix[1](0, 2), calib.camera_matrix[1](1, 2) };

            calib.projection_matrix[0] = calib.cameras[0].projection();
            calib.projection_matrix[1] = calib.cameras[1].projection();

            calib.fundamental_matrix[0] = calib.cameras[0].fundamental(calib.cameras[1]);
            calib.fundamental_matrix[1] = calib.cameras[1].fundamental(calib.cameras[0]);
        }
        else if (calib.cameras[0].distortion_model == camera_calibration::distortion_model::equidistant)
        {
            auto camera_matrix_new_0 = calib.camera_matrix[0];
            auto camera_matrix_new_1 = calib.camera_matrix[1];

            // cv::fisheye::estimateNewCameraMatrixForUndistortRectify
            //         (calib.camera_matrix[0], calib.cameras[0].distortion_coefficients, calib.cameras[0].resolution, rotation_0, camera_matrix_new_0, 1.0);
            // cv::fisheye::estimateNewCameraMatrixForUndistortRectify
            //         (calib.camera_matrix[1], calib.cameras[1].distortion_coefficients, calib.cameras[1].resolution, rotation_1, camera_matrix_new_1, 1.0);

            cv::fisheye::initUndistortRectifyMap
            (
                calib.camera_matrix[0],
                calib.cameras[0].distortion_coefficients,
                rotation_0,
                camera_matrix_new_0,
                calib.cameras[0].resolution,
                CV_32FC1,
                calib.map_x[0],
                calib.map_y[0]
            );

            cv::fisheye::initUndistortRectifyMap
            (
                calib.camera_matrix[1],
                calib.cameras[1].distortion_coefficients,
                rotation_1,
                camera_matrix_new_1,
                calib.cameras[1].resolution,
                CV_32FC1,
                calib.map_x[1],
                calib.map_y[1]
            );

            calib.camera_matrix[0] = camera_matrix_new_0;
            calib.camera_matrix[1] = camera_matrix_new_1;

            calib.cameras[0].focal_length    = { calib.camera_matrix[0](0, 0), calib.camera_matrix[0](1, 1) };
            calib.cameras[0].principal_point = { calib.camera_matrix[0](0, 2), calib.camera_matrix[0](1, 2) };
            calib.cameras[1].focal_length    = { calib.camera_matrix[1](0, 0), calib.camera_matrix[1](1, 1) };
            calib.cameras[1].principal_point = { calib.camera_matrix[1](0, 2), calib.camera_matrix[1](1, 2) };

            calib.projection_matrix[0] = calib.cameras[0].projection();
            calib.projection_matrix[1] = calib.cameras[1].projection();

            calib.fundamental_matrix[0] = calib.cameras[0].fundamental(calib.cameras[1]);
            calib.fundamental_matrix[1] = calib.cameras[1].fundamental(calib.cameras[0]);
        }
    }


    if (stereo_rectify) {}

    return calib;
}

void zenslam::calibration::print() const
{
    SPDLOG_INFO("");
    cameras[0].print();
    SPDLOG_INFO("");
    cameras[1].print();
}
