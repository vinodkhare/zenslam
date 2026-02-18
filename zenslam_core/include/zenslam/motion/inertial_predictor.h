#pragma once

#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>

#include "zenslam/frame/estimated.h"
#include "zenslam/frame/processed.h"

namespace zenslam
{
    /**
     * @brief Inertial predictor interface
     *
     * This class predicts the pose of a frame based on IMU data.
     */
    class inertial_predictor
    {
    public:
        inertial_predictor() = default;

        inertial_predictor(const cv::Vec3d& gravity_in_world, const cv::Affine3d& pose_of_cam_in_imu) :
            _pose_of_cam_in_imu(pose_of_cam_in_imu),
            _gravity_in_world(gravity_in_world)
        {
        }

        auto set_pose_of_cam_in_imu(const cv::Affine3d& pose_of_cam_in_imu) -> void { _pose_of_cam_in_imu = pose_of_cam_in_imu; }
        auto set_gravity_in_world(const cv::Vec3d& gravity_in_world) -> void { _gravity_in_world = gravity_in_world; }

        // Convenience: derive velocity from two frames (world-frame velocity)
        auto update(const frame::estimated& prev, const frame::estimated& curr) -> void
        {
            const double dt = curr.timestamp - prev.timestamp;
            if (!(dt > 0.0))
            {
                return;
            }
            _velocity_in_world = (curr.pose.translation() - prev.pose.translation()) / dt;
        }

        /**
         * @brief Predict the pose of a frame given its timestamp.
         *
         * @param estimated The current estimated frame.
         * @param processed The processed frame containing IMU integration results.
         * @return The predicted pose as a cv::Affine3d object.
         */
        [[nodiscard]] auto predict(const frame::estimated& estimated, const frame::processed& processed) const -> cv::Affine3d
        {
            const auto R_c_w = estimated.pose.inv().rotation(); // cam <- world
            const auto R_i_c = _pose_of_cam_in_imu.rotation();  // imu <- cam

            // Gravity (vector-only transforms)
            const auto gravity_in_cam = R_c_w * _gravity_in_world; // cam
            const auto gravity_in_imu = R_i_c * gravity_in_cam;    // imu

            // Velocity term v0 (world -> cam -> imu)
            const auto v_cam0 = R_c_w * _velocity_in_world; // cam
            const auto v_imu0 = R_i_c * v_cam0;             // imu

            const auto rotation    = processed.integral.delta_R;
            const auto translation = v_imu0 * processed.integral.dt + processed.integral.delta_p + gravity_in_imu * processed.integral.dt_sq_half;

            const auto delta_pose_in_imu = cv::Affine3d { rotation, translation };
            const auto delta_pose_in_cam = _pose_of_cam_in_imu.inv() * delta_pose_in_imu * _pose_of_cam_in_imu;

            return delta_pose_in_cam;
        }

    private:
        cv::Affine3d _pose_of_cam_in_imu = cv::Affine3d::Identity();  //!< Pose of camera in IMU frame (T_i_c)
        cv::Vec3d    _gravity_in_world   = cv::Vec3d(0.0, 9.81, 0.0); //!< Gravity vector in world frame
        cv::Vec3d    _velocity_in_world  = cv::Vec3d(0.0, 0.0, 0.0);  //!< Velocity in world frame
    };
} // namespace zenslam
