// Reconstructed clean implementation
#include "zenslam/triangulator.h"

#include <ranges>

#include "zenslam/utils.h"
#include "zenslam/utils_opencv.h"
#include "zenslam/utils_slam.h"

namespace
{
    auto epipolar_angles(const cv::Vec3d& translation_of_camera1_in_camera0)
    {
        return std::views::transform
        (
            [&translation_of_camera1_in_camera0](const cv::Point3d& p)
            {
                const auto v0 = cv::Vec3d(p.x, p.y, p.z);
                const auto v1 = cv::Vec3d(p.x, p.y, p.z) - translation_of_camera1_in_camera0;
                const auto n  = cv::norm(v0) * cv::norm(v1);
                return n < 1e-12
                           ? 0.0
                           : std::abs(std::acos(std::clamp(v0.dot(v1) / n, -1.0, 1.0))) * 180 / CV_PI;
            }
        );
    }
}

namespace zenslam
{
    triangulator::triangulator(const calibration& calib, const slam_options& opts)
        : _calibration(calib), _options(opts)
    {
    }

    auto triangulator::triangulate_keypoints(const map<keypoint>& keypoints_0,const map<keypoint>& keypoints_1) const -> std::vector<point3d>
    {
        const auto& points2f_0 = utils::to_points(keypoints_0.values_matched(keypoints_1) | std::ranges::to<std::vector>());
        const auto& points2f_1 = utils::to_points(keypoints_1.values_matched(keypoints_0) | std::ranges::to<std::vector>());

        const auto& points3d_cv = utils::triangulate_points
        (
            points2f_0,
            points2f_1,
            _calibration.projection_matrix[0],
            _calibration.projection_matrix[1]
        );

        const auto& indices     = keypoints_0.keys_matched(keypoints_1) | std::ranges::to<std::vector>();
        const auto& descriptors = keypoints_0.values_matched(keypoints_1)
            | std::views::transform([](const keypoint& kp) { return kp.descriptor; })
            | std::ranges::to<std::vector>();

        const auto& points3d_all = point3d::create(points3d_cv, indices, descriptors);

        const auto& points2f_0_back = utils::project(points3d_cv, _calibration.projection_matrix[0]);
        const auto& points2f_1_back = utils::project(points3d_cv, _calibration.projection_matrix[1]);

        const auto& errors_0 = utils::vecnorm(points2f_0_back - points2f_0);
        const auto& errors_1 = utils::vecnorm(points2f_1_back - points2f_1);

        const auto& angles = points3d_all
            | epipolar_angles(_calibration.cameras[1].pose_in_cam0.translation())
            | std::ranges::to<std::vector>();

        std::vector<point3d> points3d;
        for (size_t i = 0; i < points3d_all.size(); ++i)
        {
            if (points3d_all[i].z > 1 &&
                errors_0[i] < _options.triangulation_reprojection_threshold &&
                errors_1[i] < _options.triangulation_reprojection_threshold &&
                angles[i] > 0.25 && angles[i] < 180 - 0.25)
            {
                points3d.emplace_back(points3d_all[i]);
            }
        }

        return points3d;
    }
}
