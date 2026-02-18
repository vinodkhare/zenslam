#pragma once

#include <opencv2/core/affine.hpp>
#include <opencv2/core/matx.hpp>

#include "zenslam/frame/estimated.h"

namespace zenslam
{
    class motion_predictor
    {
    public:
        /**
         * @brief Predict the next pose and update the processed frame with the predicted pose
         *
         * This function uses the internal motion model to predict the next pose based on the
         * last known pose and the elapsed time. It updates the provided processed frame with
         * the predicted pose.
         *
         * @param estimated_0 The last estimated frame containing the current pose
         * @param processed_1 The processed frame to be updated with the predicted pose
         */
        auto predict(const frame::estimated& estimated_0, const frame::processed& processed_1) const -> cv::Affine3d;

        /**
         * @brief Update the motion model based on two estimated frames
         *
         * This function updates the internal velocity and acceleration estimates
         * based on the poses and timestamps of two consecutive estimated frames.
         *
         * @param estimated_0 The previous estimated frame
         * @param estimated_1 The current estimated frame
         */
        auto update(const frame::estimated& estimated_0, const frame::estimated& estimated_1) -> void;

    private:
        cv::Vec3d _vel { cv::Vec3d::zeros() };
        cv::Vec3d _acc { cv::Vec3d::zeros() };
        cv::Vec3d _vel_ang { cv::Vec3d::zeros() };
        cv::Vec3d _acc_ang { cv::Vec3d::zeros() };
    };
}
