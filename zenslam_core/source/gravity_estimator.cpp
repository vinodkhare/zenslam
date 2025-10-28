#include "zenslam/gravity_estimator.h"

auto zenslam::gravity_estimator::add(const frame::estimated& f) -> void
{
    // Transform camera-frame down axis into world frame and store
    const auto Rwc = f.pose.rotation();

    const cv::Vec3d down_w
    {
        Rwc(0,0) * _axis_cam_down[0] + Rwc(0,1) * _axis_cam_down[1] + Rwc(0,2) * _axis_cam_down[2],
        Rwc(1,0) * _axis_cam_down[0] + Rwc(1,1) * _axis_cam_down[1] + Rwc(1,2) * _axis_cam_down[2],
        Rwc(2,0) * _axis_cam_down[0] + Rwc(2,1) * _axis_cam_down[1] + Rwc(2,2) * _axis_cam_down[2]
    };

    const auto n = cv::norm(down_w);
    if (n > 0)
    {
        _samples.emplace_back(down_w / n);
    }
}

auto zenslam::gravity_estimator::estimate(const double magnitude) const -> cv::Vec3d
{
    if (_samples.empty())
    {
        // Fallback to default axis transformed by identity
        return magnitude * _axis_cam_down;
    }

    cv::Vec3d avg { 0.0, 0.0, 0.0 };
    for (const auto& s : _samples)
    {
        avg += s;
    }

    const auto n = cv::norm(avg);
    if (n == 0.0)
    {
        return magnitude * _axis_cam_down;
    }

    return (avg / n) * magnitude;
}
