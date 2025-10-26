#include "zenslam/motion.h"

#include <opencv2/calib3d.hpp>

auto zenslam::motion::predict(const cv::Affine3d& pose_0, const double dt) const -> cv::Affine3d
{
    auto translation_1 = pose_0.translation() + _velocity_1 * dt + 0.5 * _acceleration_1 * dt * dt;

    cv::Matx33d rotation_d { };
    cv::Rodrigues(_angular_velocity_1 * dt + 0.5 * _angular_acceleration_1 * dt * dt, rotation_d);

    return { pose_0.rotation() * rotation_d, translation_1 };
}

auto zenslam::motion::update(const cv::Affine3d& pose_0, const cv::Affine3d& pose_1, const double dt) -> void
{
    _velocity_0 = _velocity_1;
    _velocity_1 = (pose_1.translation() - pose_0.translation()) / dt;

    _acceleration_0 = _acceleration_1;
    _acceleration_1 = (_velocity_1 - _velocity_0) / dt;

    cv::Vec3d rvec { };
    cv::Rodrigues(pose_0.rotation().inv() * pose_1.rotation(), rvec);

    _angular_velocity_0 = _angular_velocity_1;
    _angular_velocity_1 = rvec / dt;

    _angular_acceleration_0 = _angular_acceleration_1;
    _angular_acceleration_1 = (_angular_velocity_1 - _angular_velocity_0) / dt;
}