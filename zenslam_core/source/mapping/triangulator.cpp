// Reconstructed clean implementation
#include "zenslam/mapping/triangulator.h"

#include <ranges>
#include <utility>

#include "zenslam/mapping/triangulation_utils.h"
#include "zenslam/utils/utils.h"
#include "zenslam/utils/utils_opencv.h"
#include "zenslam/utils/utils_slam.h"

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
    triangulator::triangulator(calibration calib, const slam_options& opts) :
        _calibration(std::move(calib)),
        _options(opts)
    {
    }

    auto triangulator::triangulate_keypoints(const map<keypoint>& keypoints_0, const map<keypoint>& keypoints_1, const cv::Mat& color_image) const -> point3d_cloud
    {
        std::vector<keypoint> keypoints_0_filtered { };
        std::vector<keypoint> keypoints_1_filtered { };

        if (_options.triangulation.filter_epipolar)
        {
            const auto& filtered_matches = filter_epipolar(keypoints_0, keypoints_1);

            for (const auto& [kp0, kp1] : filtered_matches)
            {
                keypoints_0_filtered.emplace_back(kp0);
                keypoints_1_filtered.emplace_back(kp1);
            }
        }
        else
        {
            keypoints_0_filtered = keypoints_0.values_matched(keypoints_1) | std::ranges::to<std::vector>();
            keypoints_1_filtered = keypoints_1.values_matched(keypoints_0) | std::ranges::to<std::vector>();
        }

        const auto& points2f_0 = utils::to_points(keypoints_0_filtered);
        const auto& points2f_1 = utils::to_points(keypoints_1_filtered);

        const auto& points3d_cv = utils::triangulate_points
        (
            points2f_0,
            points2f_1,
            _calibration.projection_matrix[0],
            _calibration.projection_matrix[1]
        );

        const auto& indices     = keypoints_0_filtered | std::views::transform([](const keypoint& kp) { return kp.index; }) | std::ranges::to<std::vector>();
        const auto& descriptors = keypoints_0_filtered | std::views::transform([](const keypoint& kp) { return kp.descriptor; }) | std::ranges::to<std::vector>();

        // Extract colors from the color image if provided
        std::vector<cv::Vec3b> colors;
        if (!color_image.empty())
        {
            colors.reserve(keypoints_0_filtered.size());

            for (const auto& kp : keypoints_0_filtered)
            {
                auto x = static_cast<int>(std::round(kp.pt.x));
                auto y = static_cast<int>(std::round(kp.pt.y));

                // Clamp to image bounds
                x = std::max(0, std::min(x, color_image.cols - 1));
                y = std::max(0, std::min(y, color_image.rows - 1));

                if (color_image.type() == CV_8UC3)
                {
                    colors.push_back(color_image.at<cv::Vec3b>(y, x));
                }
                else if (color_image.type() == CV_8UC1)
                {
                    const auto intensity = color_image.at<uchar>(y, x);
                    colors.emplace_back(intensity, intensity, intensity); // Grayscale to RGB
                }
                else
                {
                    colors.emplace_back(255, 255, 255); // Default to black if unsupported type
                }
            }
        }

        const auto& points3d_all = point3d::create(points3d_cv, indices, descriptors, colors);

        const auto& points2f_0_back = utils::project(points3d_cv, _calibration.projection_matrix[0]);
        const auto& points2f_1_back = utils::project(points3d_cv, _calibration.projection_matrix[1]);

        const auto& errors_0 = utils::vecnorm(points2f_0_back - points2f_0);
        const auto& errors_1 = utils::vecnorm(points2f_1_back - points2f_1);

        const auto& angles = points3d_all
            | epipolar_angles(_calibration.cameras[1].pose_in_cam0.translation())
            | std::ranges::to<std::vector>();

        point3d_cloud points3d;
        for (size_t i = 0; i < points3d_all.size(); ++i)
        {
            if (points3d_all[i].z > 0 &&
                cv::norm(points3d_all[i]) > _options.triangulation.min_depth &&
                cv::norm(points3d_all[i]) < _options.triangulation.max_depth &&
                errors_0[i] < _options.triangulation.reprojection_threshold &&
                errors_1[i] < _options.triangulation.reprojection_threshold &&
                angles[i] > 0.25 && angles[i] < 180 - 0.25)
            {
                points3d.add(points3d_all[i]);
            }
        }

        return points3d;
    }

    auto triangulator::triangulate_keylines(const map<keyline>& keylines_0, const map<keyline>& keylines_1, const cv::Mat& color_image) const -> line3d_cloud
    {
        const auto vec = utils::triangulate_keylines
        (
            keylines_0,
            keylines_1,
            _calibration.projection_matrix[0],
            _calibration.projection_matrix[1],
            _options,
            _calibration.cameras[1].pose_in_cam0.translation()
        );

        line3d_cloud lines3d { };
        lines3d.add(vec);

        return lines3d;
    }

    auto triangulator::filter_epipolar(const map<keypoint>& keypoints_0, const map<keypoint>& keypoints_1) const -> std::vector<std::pair<keypoint, keypoint>>
    {
        const auto& keypoints_0_matched = keypoints_0.values_matched(keypoints_1) | std::ranges::to<std::vector>();
        const auto& keypoints_1_matched = keypoints_1.values_matched(keypoints_0) | std::ranges::to<std::vector>();

        const auto& error = std::views::zip(keypoints_0_matched, keypoints_1_matched) | std::views::transform
        (
            [&](const auto& pair)
            {
                const auto& [kp0, kp1] = pair;

                const auto pt0 = cv::Vec3d(kp0.pt.x, kp0.pt.y, 1);
                const auto pt1 = cv::Vec3d(kp1.pt.x, kp1.pt.y, 1);

                return (pt1.t() * _calibration.fundamental_matrix[0] * pt0)[0];
            }
        ) | std::ranges::to<std::vector>();

        return std::views::zip(keypoints_0_matched, keypoints_1_matched, error)
            | std::views::filter
            (
                [&](const auto& tuple)
                {
                    const auto& [kp0, kp1, err] = tuple;
                    return std::abs(err) < _options.triangulation.epipolar_threshold; // Epipolar error threshold (tunable)
                }
            )
            | std::views::transform
            (
                [](const auto& tuple)
                {
                    const auto& [kp0, kp1, err] = tuple;
                    return std::make_pair(kp0, kp1);
                }
            )
            | std::ranges::to<std::vector>();
    }
}
