#include "zenslam/mapping/triangulation_utils.h"

#include <algorithm>
#include <cmath>
#include <ranges>
#include <vector>

#include <opencv2/calib3d.hpp>

#include "zenslam/types/keyline.h"
#include "zenslam/types/line3d.h"
#include "zenslam/utils/utils.h"
#include "zenslam/utils/utils_opencv.h"
#include "zenslam/utils/utils_slam.h"

namespace
{
    // Helper view transformations for extracting line endpoints
    auto points_0()
    {
        return std::views::transform(
            [](const zenslam::keyline& kl) { return kl.getStartPoint(); });
    }

    auto points_1()
    {
        return std::views::transform(
            [](const zenslam::keyline& kl) { return kl.getEndPoint(); });
    }

    // Helper for computing epipolar angles
    auto epipolar_angles(const cv::Vec3d& translation_of_camera1_in_camera0)
    {
        return std::views::transform(
            [&translation_of_camera1_in_camera0](const cv::Point3d& p)
            {
                const auto vec_to_point_0 = cv::Vec3d(p.x, p.y, p.z);
                const auto vec_to_point_1 =
                    cv::Vec3d(p.x, p.y, p.z) - translation_of_camera1_in_camera0;
                const auto norm_prod = cv::norm(vec_to_point_0) * cv::norm(vec_to_point_1);
                return norm_prod < 1e-12
                           ? 0.0
                           : std::abs(std::acos(std::clamp(
                               vec_to_point_0.dot(vec_to_point_1) / norm_prod,
                               -1.0,
                               1.0))) *
                           180 / CV_PI;
            });
    }
}

auto zenslam::utils::triangulate_keylines(
    const map<keyline>& keylines_0,
    const map<keyline>& keylines_1,
    const cv::Matx34d&  projection_0,
    const cv::Matx34d&  projection_1,
    const slam_options& options,
    const cv::Vec3d&    translation_of_camera1_in_camera0) -> std::vector<line3d>
{
    const auto& points2f_0_0 = keylines_0.values_matched(keylines_1) | points_0() |
        std::ranges::to<std::vector>();
    const auto& points2f_0_1 = keylines_0.values_matched(keylines_1) | points_1() |
        std::ranges::to<std::vector>();
    const auto& points2f_1_0 = keylines_1.values_matched(keylines_0) | points_0() |
        std::ranges::to<std::vector>();
    const auto& points2f_1_1 = keylines_1.values_matched(keylines_0) | points_1() |
        std::ranges::to<std::vector>();

    const auto& points3d_cv_0 =
        triangulate_points(points2f_0_0, points2f_1_0, projection_0, projection_1);
    const auto& points3d_cv_1 =
        triangulate_points(points2f_0_1, points2f_1_1, projection_0, projection_1);

    const auto& points2f_0_0_back = project(points3d_cv_0, projection_0);
    const auto& points2f_1_0_back = project(points3d_cv_0, projection_1);
    const auto& points2f_0_1_back = project(points3d_cv_1, projection_0);
    const auto& points2f_1_1_back = project(points3d_cv_1, projection_1);

    const auto& errors_0_0 = vecnorm(points2f_0_0_back - points2f_0_0);
    const auto& errors_1_0 = vecnorm(points2f_1_0_back - points2f_1_0);
    const auto& errors_0_1 = vecnorm(points2f_0_1_back - points2f_0_1);
    const auto& errors_1_1 = vecnorm(points2f_1_1_back - points2f_1_1);

    const auto& angles_0 = points3d_cv_0 | epipolar_angles(translation_of_camera1_in_camera0) |
        std::ranges::to<std::vector>();
    const auto& angles_1 = points3d_cv_1 | epipolar_angles(translation_of_camera1_in_camera0) |
        std::ranges::to<std::vector>();

    const auto& indices_0 =
        keylines_0.keys_matched(keylines_1) | std::ranges::to<std::vector>();

    std::vector<double> ang;
    for (auto i = 0; i < points3d_cv_0.size(); ++i)
    {
        auto       vector_0  = (points3d_cv_0[i] + points3d_cv_1[i]) / 2.0;
        auto       vector_1  = points3d_cv_0[i] - points3d_cv_1[i];
        const auto norm_prod = cv::norm(vector_0) * cv::norm(vector_1);
        auto       angle     = norm_prod < 1e-12
                                   ? 0.0
                                   : std::abs(std::acos(std::clamp(
                                       vector_0.dot(vector_1) / norm_prod,
                                       -1.0,
                                       1.0))) *
                                   180 / CV_PI;
        ang.push_back(angle);
    }

    std::vector<line3d> lines3d { };
    for (auto i = 0; i < points3d_cv_0.size(); ++i)
    {
        const auto depth_0 = cv::norm(points3d_cv_0[i]);
        const auto depth_1 = cv::norm(points3d_cv_1[i]);

        if (points3d_cv_0[i].z > 0 && points3d_cv_1[i].z > 0 &&
            depth_0 > options.triangulation.min_depth &&
            depth_0 < options.triangulation.max_depth &&
            depth_1 > options.triangulation.min_depth &&
            depth_1 < options.triangulation.max_depth &&
            errors_0_0[i] < options.triangulation.reprojection_threshold &&
            errors_1_0[i] < options.triangulation.reprojection_threshold &&
            errors_0_1[i] < options.triangulation.reprojection_threshold &&
            errors_1_1[i] < options.triangulation.reprojection_threshold &&
            angles_0[i] > 0.25 && angles_0[i] < 180 - 0.25 && angles_1[i] > 0.25 &&
            angles_1[i] < 180 - 0.25 && ang[i] > 45.0 && ang[i] < 135.0)
        {
            lines3d.emplace_back(
                std::array { points3d_cv_0[i], points3d_cv_1[i] },
                indices_0[i]);
        }
    }

    return lines3d;
}

auto zenslam::utils::triangulate_points(
    const std::vector<cv::Point2f>& points2f_0,
    const std::vector<cv::Point2f>& points2f_1,
    const cv::Matx34d&              projection_0,
    const cv::Matx34d&              projection_1) -> std::vector<cv::Point3d>
{
    if (points2f_0.empty() || points2f_1.empty())
    {
        return { };
    }

    cv::Mat points4d { };
    cv::triangulatePoints(projection_0, projection_1, points2f_0, points2f_1, points4d);

    std::vector<cv::Point3d> points3d { };
    for (auto i = 0; i < points4d.cols; ++i)
    {
        cv::Vec4d X = points4d.col(i);
        if (std::abs(X[3]) > 1E-9)
            points3d.emplace_back(X[0] / X[3], X[1] / X[3], X[2] / X[3]);
        else
            points3d.emplace_back(0, 0, 0);
    }

    return points3d;
}
