#pragma once

#include <opencv2/core/affine.hpp>
#include <opencv2/core/matx.hpp>

namespace zenslam
{
    class motion
    {
    public:
        auto predict(const cv::Affine3d &pose_0, double dt) const -> cv::Affine3d;
        auto update(const cv::Affine3d &pose_0, const cv::Affine3d &pose_1, double dt) -> void;

    private:
        cv::Vec3d _velocity_1 { cv::Vec3d::zeros() };
        cv::Vec3d _velocity_0 { cv::Vec3d::zeros() };
        cv::Vec3d _acceleration_0 { cv::Vec3d::zeros() };
        cv::Vec3d _acceleration_1 { cv::Vec3d::zeros() };
        cv::Vec3d _angular_velocity_0 { cv::Vec3d::zeros() };
        cv::Vec3d _angular_velocity_1 { cv::Vec3d::zeros() };
        cv::Vec3d _angular_acceleration_0 { cv::Vec3d::zeros() };
        cv::Vec3d _angular_acceleration_1 { cv::Vec3d::zeros() };
    };
}
