#include "zenslam/motion/gravity_estimator.h"

#include <algorithm>
#include <spdlog/spdlog.h>

auto zenslam::gravity_estimator::add(const frame::estimated& estimated_curr, const cv::Affine3d& T_cam_imu) -> void
{
    // Add current pose to history
    _pose_history.push_back({ estimated_curr.timestamp, estimated_curr.pose.translation(), estimated_curr.pose.rotation() });

    // Keep only last 3 poses
    if (_pose_history.size() > _history_size)
    {
        _pose_history.pop_front();
    }

    // Need at least 3 poses to estimate acceleration (central difference)
    if (_pose_history.size() < _history_size)
    {
        return;
    }

    // Get three consecutive poses for central difference acceleration
    const auto& p0 = _pose_history[0];
    const auto& p1 = _pose_history[1];
    const auto& p2 = _pose_history[2];

    const auto dt01 = p1.timestamp - p0.timestamp;
    const auto dt12 = p2.timestamp - p1.timestamp;

    if (dt01 <= 0.0 || dt12 <= 0.0)
    {
        return;
    }

    // Compute velocities in world frame
    const auto v01 = (p1.position - p0.position) / dt01;
    const auto v12 = (p2.position - p1.position) / dt12;

    // Compute acceleration in world frame using central difference
    const auto dt_avg      = (dt01 + dt12) / 2.0;
    const auto a_cam_world = (v12 - v01) / dt_avg;

    // Transform acceleration from world to camera frame at p1 (middle frame)
    const auto a_cam = p1.rotation.t() * a_cam_world;

    // Transform from camera to IMU frame
    const auto R_imu_cam  = T_cam_imu.rotation().t();
    const auto a_imu_true = R_imu_cam * a_cam;

    // Get average measured acceleration from IMU data between frames
    if (estimated_curr.imu_data.empty())
    {
        SPDLOG_WARN("No IMU data available for gravity estimation");
        return;
    }

    cv::Vec3d a_measured_sum { 0.0, 0.0, 0.0 };
    for (const auto& imu : estimated_curr.imu_data)
    {
        a_measured_sum += imu.acc;
    }
    const auto a_imu_measured = a_measured_sum / static_cast<double>(estimated_curr.imu_data.size());

    // The residual is gravity: a_measured = a_true - g  =>  g = a_true - a_measured
    const auto g_imu = a_imu_true - a_imu_measured;

    // Transform gravity back to world frame
    const auto R_cam_imu = T_cam_imu.rotation();
    const auto g_cam     = R_cam_imu * g_imu;
    const auto g_world   = p1.rotation * g_cam;

    // Sanity check: gravity magnitude should be around 9.8 m/s²
    const auto g_mag = cv::norm(g_world);
    if (g_mag > 6.0 && g_mag < 15.0) // Allow some noise but reject outliers
    {
        _gravity_samples.push_back(g_world);
        SPDLOG_DEBUG("Gravity sample: [{:.3f}, {:.3f}, {:.3f}], mag: {:.3f}", g_world[0], g_world[1], g_world[2], g_mag);
    }
    else
    {
        SPDLOG_DEBUG("Rejected gravity sample with magnitude: {:.3f}", g_mag);
    }
}

auto zenslam::gravity_estimator::estimate() const -> cv::Vec3d
{
    if (_gravity_samples.empty())
    {
        // Default fallback
        return { 0.0, 9.81, 0.0 };
    }

    // Use median for robustness against outliers
    // Compute component-wise medians
    std::vector<double> x_vals, y_vals, z_vals;
    x_vals.reserve(_gravity_samples.size());
    y_vals.reserve(_gravity_samples.size());
    z_vals.reserve(_gravity_samples.size());

    for (const auto& g : _gravity_samples)
    {
        x_vals.push_back(g[0]);
        y_vals.push_back(g[1]);
        z_vals.push_back(g[2]);
    }

    auto median = [](std::vector<double>& vals) -> double
    {
        const auto n = vals.size();
        std::ranges::nth_element(vals, vals.begin() + n / 2);
        if (n % 2 == 0)
        {
            const auto m1 = vals[n / 2];
            std::ranges::nth_element(vals, vals.begin() + n / 2ul - 1);
            return (m1 + vals[n / 2 - 1]) / 2.0;
        }
        return vals[n / 2];
    };

    const cv::Vec3d g_median(median(x_vals), median(y_vals), median(z_vals));

    SPDLOG_INFO
    (
        "Gravity estimate from {} samples: [{:.3f}, {:.3f}, {:.3f}], mag: {:.3f}",
        _gravity_samples.size(),
        g_median[0],
        g_median[1],
        g_median[2],
        cv::norm(g_median)
    );

    return g_median;
}
