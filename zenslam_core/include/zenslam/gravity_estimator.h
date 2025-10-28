#pragma once

#include <vector>

#include <opencv2/core/matx.hpp>

#include "frame/estimated.h"

namespace zenslam
{
    /**
     * @brief Estimate gravity vector in world frame from early-frame poses.
     *
     * Assumes a known "down" direction in the camera frame (default OpenCV camera Y+).
     * For each input pose, transforms this camera-frame down-axis into the world frame
     * and accumulates samples. The estimate is the normalized average direction scaled
     * by the specified magnitude (default 9.81 m/s^2).
     */
    class gravity_estimator
    {
    public:
        gravity_estimator() = default;

        explicit gravity_estimator(const cv::Vec3d& cam_down_axis) : _axis_cam_down(normalize(cam_down_axis))
        {
        }

        // Provide an estimated frame (pose must have valid rotation)
        auto add(const frame::estimated& f) -> void;

        // Return number of accumulated samples
        [[nodiscard]] auto count() const -> size_t { return _samples.size(); }

        // Return gravity vector in world frame with specified magnitude (default 9.81)
        [[nodiscard]] auto estimate(double magnitude = 9.81) const -> cv::Vec3d;

        // Reset accumulated samples
        auto reset() -> void { _samples.clear(); }

        // Change the assumed camera-frame down axis (will normalize internally)
        auto set_axis_cam_down(const cv::Vec3d& axis) -> void { _axis_cam_down = normalize(axis); }

    private:
        static auto normalize(const cv::Vec3d& v) -> cv::Vec3d
        {
            const auto n = cv::norm(v);
            return n > 0 ? v / n : cv::Vec3d(0, 1, 0);
        }

        cv::Vec3d              _axis_cam_down { 0.0, 1.0, 0.0 }; // OpenCV camera Y+ points down
        std::vector<cv::Vec3d> _samples;                         // world-frame down-axis samples
    };
}
