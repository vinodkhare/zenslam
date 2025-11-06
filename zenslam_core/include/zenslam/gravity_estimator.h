#pragma once

#include <deque>
#include <vector>

#include <opencv2/core/affine.hpp>
#include <opencv2/core/matx.hpp>

#include "frame/estimated.h"
#include "frame/processed.h"

namespace zenslam
{
    /**
     * @brief Estimate gravity vector by comparing visual odometry acceleration with IMU measurements.
     *
     * This estimator uses a residual-based approach:
     * 1. Computes acceleration from camera poses (visual odometry)
     * 2. Transforms camera acceleration to IMU frame
     * 3. Compares with measured IMU acceleration
     * 4. The residual is the gravity vector (since a_measured = a_true - g)
     *
     * Does NOT assume static initialization - works during dynamic motion.
     */
    class gravity_estimator
    {
    public:
        gravity_estimator() = default;

        /**
         * Add a pair of consecutive frames for gravity estimation.
         * Requires at least 3 frames to compute acceleration (need velocity from 2 frames).
         */
        auto add
        (
            const frame::estimated& estimated_curr,
            const cv::Affine3d&     T_cam_imu
        ) -> void;

        /**
         * Return gravity vector in world frame.
         * Returns the median of accumulated gravity samples for robustness.
         */
        [[nodiscard]] auto estimate() const -> cv::Vec3d;

        /**
         * Check if we have enough samples for a reliable estimate.
         */
        [[nodiscard]] auto has_estimate() const -> bool { return _gravity_samples.size() >= 10; }

        /**
         * Return number of accumulated samples.
         */
        [[nodiscard]] auto count() const -> size_t { return _gravity_samples.size(); }

        /**
         * Reset accumulated samples.
         */
        auto reset() -> void
        {
            _gravity_samples.clear();
            _pose_history.clear();
        }

    private:
        struct pose_sample
        {
            double      timestamp;
            cv::Vec3d   position;
            cv::Matx33d rotation;
        };

        std::vector<cv::Vec3d>  _gravity_samples;
        std::deque<pose_sample> _pose_history; // Keep last 3 poses for acceleration estimation
        static constexpr size_t _history_size = 3;
    };
}
