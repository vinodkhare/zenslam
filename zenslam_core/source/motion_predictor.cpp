#include "zenslam/motion_predictor.h"

#include <opencv2/calib3d.hpp>

auto zenslam::motion_predictor::predict(const frame::estimated& estimated_0, const frame::processed& processed_1) const -> cv::Affine3d
{
    const auto dt          = isnan(estimated_0.timestamp) ? 0 : processed_1.timestamp - estimated_0.timestamp;
    const auto translation = _vel * dt + 0.5 * _acc * dt * dt;

    cv::Matx33d rotation { };
    cv::Rodrigues(_vel_ang * dt + 0.5 * _acc_ang * dt * dt, rotation);

    return { rotation, translation };
}

auto zenslam::motion_predictor::update(const frame::estimated& estimated_0, const frame::estimated& estimated_1) -> void
{
    const auto dt = estimated_1.timestamp - estimated_0.timestamp;

    if (isnan(dt))
    {
        return;
    }

    const auto vel = (estimated_1.pose.translation() - estimated_0.pose.translation()) / dt;
    const auto acc = (vel - _vel) / dt;

    cv::Vec3d rvec { };
    cv::Rodrigues(estimated_0.pose.rotation().inv() * estimated_1.pose.rotation(), rvec);

    const auto vel_ang = rvec / dt;
    const auto acc_ang = (vel_ang - _vel_ang) / dt;

    _vel     = (vel + _vel) / 2.0;
    _acc     = (acc + _acc) / 2.0;
    _vel_ang = (vel_ang + _vel_ang) / 2.0;
    _acc_ang = (acc_ang + _acc_ang) / 2.0;
}
