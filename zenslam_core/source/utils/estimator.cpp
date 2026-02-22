#include "zenslam/utils/estimator.h"

#include <algorithm>
#include <array>
#include <gsl/narrow>

#include <opencv2/calib3d.hpp>
#include <spdlog/spdlog.h>

#include "zenslam/pose_estimation/combined_estimator.h"
#include "zenslam/pose_estimation/line_estimator.h"
#include "zenslam/pose_estimation/point_estimator.h"
#include "zenslam/pose_estimation/pose_fusion.h"
#include "zenslam/utils/rigid_transform.h"

namespace zenslam
{
    estimator::estimator(calibration calib, slam_options opts) :
        _calibration(std::move(calib)),
        _options(opts),
        _point_estimator(std::make_unique<pose_estimation::point_estimator>(_calibration, _options)),
        _line_estimator(std::make_unique<pose_estimation::line_estimator>(_calibration, _options)),
        _combined_estimator(std::make_unique<pose_estimation::combined_estimator>(_calibration, _options))
    {
    }

    estimator::~estimator()                               = default;
    estimator::estimator(estimator&&) noexcept            = default;
    estimator& estimator::operator=(estimator&&) noexcept = default;

    auto estimator::estimate_pose
    (
        const std::map<size_t, point3d>& points3d_0,
        const frame::tracked&            tracked_1
    ) const
        -> estimate_pose_result
    {
        estimate_pose_result result{};

        // Combined 3D-3D estimation using both points and lines
        if (const auto pose = _combined_estimator->estimate_3d3d
        (
            points3d_0,
            tracked_1.points3d,
            std::map<size_t, line3d>{},
            // No previous lines in this overload
            tracked_1.lines3d
        ))
        {
            result.pose_3d3d       = *pose;
            result.pose_3d3d_lines = *pose; // Backward compatibility
        }

        // Combined 3D-2D estimation using both points and lines
        if (const auto pose = _combined_estimator->estimate_3d2d
        (
            points3d_0,
            tracked_1.keypoints[0],
            tracked_1.lines3d,
            tracked_1.keylines[0]
        ))
        {
            result.pose_3d2d       = *pose;
            result.pose_3d2d_lines = *pose; // Backward compatibility
        }

        // Choose the pose with more inliers
        const auto inliers_3d3d = result.pose_3d3d.inliers.size();
        const auto inliers_3d2d = result.pose_3d2d.inliers.size();

        if (inliers_3d3d >= inliers_3d2d && inliers_3d3d > 0)
        {
            result.chosen_pose  = result.pose_3d3d.pose;
            result.chosen_count = inliers_3d3d;
        }
        else if (inliers_3d2d > 0)
        {
            result.chosen_pose  = result.pose_3d2d.pose;
            result.chosen_count = inliers_3d2d;
        }
        else
        {
            result.chosen_pose  = cv::Affine3d::Identity();
            result.chosen_count = 0;
        }

        return result;
    }

    auto estimator::estimate_pose
    (
        const frame::estimated& frame_0,
        const frame::tracked&   tracked_1
    ) const
        -> estimate_pose_result
    {
        estimate_pose_result result{};

        // Combined 3D-3D estimation
        if
        (
            auto pose = _combined_estimator->estimate_3d3d
            (
                frame_0.points3d,
                tracked_1.points3d,
                frame_0.lines3d,
                tracked_1.lines3d
            )
        )
        {
            result.pose_3d3d       = *pose;
            result.pose_3d3d_lines = *pose; // Backward compatibility
        }

        // Combined 3D-2D estimation
        if (auto pose = _combined_estimator->estimate_3d2d
        (
            frame_0.points3d,
            tracked_1.keypoints[0],
            frame_0.lines3d,
            tracked_1.keylines[0]
        ))
        {
            result.pose_3d2d       = *pose;
            result.pose_3d2d_lines = *pose; // Backward compatibility
        }

        // Combined 2D-2D estimation
        if (auto pose = _combined_estimator->estimate_2d2d
        (
            frame_0.keypoints[0],
            tracked_1.keypoints[0],
            frame_0.points3d,
            frame_0.lines3d,
            frame_0.keylines[0],
            tracked_1.keylines[0]
        ))
        {
            result.pose_2d2d = *pose;
        }

        // Choose best pose based on inlier count with tie-breaker preference
        const auto in3d3d = result.pose_3d3d.inliers.size();
        const auto in3d2d = result.pose_3d2d.inliers.size();
        const auto in2d2d = result.pose_2d2d.inliers.size();

        size_t best_inliers = 0;
        auto   best_pose    = cv::Affine3d::Identity();

        // Prefer 3D-3D > 3D-2D > 2D-2D when inlier counts are equal
        if (in3d3d > best_inliers)
        {
            best_inliers = in3d3d;
            best_pose    = result.pose_3d3d.pose;
        }

        if (in3d2d > best_inliers || (in3d2d == best_inliers && best_inliers > 0 && in3d2d > 0))
        {
            best_inliers = in3d2d;
            best_pose    = result.pose_3d2d.pose;
        }

        if (in2d2d > best_inliers || (in2d2d == best_inliers && best_inliers > 0 && in2d2d > 0))
        {
            best_inliers = in2d2d;
            best_pose    = result.pose_2d2d.pose;
        }

        result.chosen_pose  = best_pose;
        result.chosen_count = best_inliers;

        return result;
    }

    pose_data estimator::estimate_pose_3d2d(const frame::estimated& frame_0, const frame::tracked& tracked_1, const size_t& camera_index) const
    {
        // First we estimate the 3D-2D pose
        auto correspondences_3d2d
            = frame_0.points3d
            | std::views::values
            | std::views::filter([&tracked_1, &camera_index](const auto& point3d) { return tracked_1.keypoints[camera_index].contains(point3d.index); })
            | std::views::transform([&tracked_1, &camera_index](const auto& point3d) { return std::make_pair(point3d, tracked_1.keypoints[camera_index].at(point3d.index)); })
            | std::ranges::to<std::vector>();

        const auto& keylines = tracked_1.keylines[camera_index];
        const auto  line_correspondences
            = frame_0.lines3d
            | std::views::values
            | std::views::filter([&keylines](const auto& line) { return keylines.contains(line.index); })
            | std::views::transform([&keylines](const auto& line)
            {
                const auto& keyline = keylines.at(line.index);

                point3d endpoint_0 {};
                point3d endpoint_1 {};
                static_cast<cv::Point3d&>(endpoint_0) = line[0];
                static_cast<cv::Point3d&>(endpoint_1) = line[1];
                endpoint_0.index = line.index;
                endpoint_1.index = line.index;

                keypoint keypoint_0 {};
                keypoint keypoint_1 {};
                keypoint_0.pt    = cv::Point2f(gsl::narrow_cast<float>(keyline.startPointX), gsl::narrow_cast<float>(keyline.startPointY));
                keypoint_1.pt    = cv::Point2f(gsl::narrow_cast<float>(keyline.endPointX), gsl::narrow_cast<float>(keyline.endPointY));
                keypoint_0.index = line.index;
                keypoint_1.index = line.index;

                return std::array<std::pair<point3d, keypoint>, 2>
                {
                    std::pair { endpoint_0, keypoint_0 },
                    std::pair { endpoint_1, keypoint_1 }
                };
            })
            | std::ranges::to<std::vector>();

        std::ranges::for_each(line_correspondences, [&correspondences_3d2d](const auto& pair_set)
        {
            correspondences_3d2d.insert(correspondences_3d2d.end(), pair_set.begin(), pair_set.end());
        });

        if (correspondences_3d2d.size() < 4)
        {
            throw std::logic_error("Not enough 3D-2D correspondences for PnP RANSAC: " + std::to_string(correspondences_3d2d.size()) + " (need >= 4)");
        }

        pose_data pose{};
        try
        {
            pose = solvepnp_ransac(correspondences_3d2d);

            if (_options.pnp.use_refinement && pose.inliers.size() >= _options.pnp.min_refinement_inliers)
            {
                const auto& inlier_correspondences
                    = pose.inliers
                    | std::views::transform([&correspondences_3d2d](const auto& i) { return correspondences_3d2d[i]; })
                    | std::ranges::to<std::vector>();

                pose = solvepnp_refinelm(inlier_correspondences, pose);
            }
        }
        catch (const std::exception& e)
        {
            throw std::logic_error(std::string("Solve PnP (RANSAC) failed:\n") + e.what());
        }

        return pose;
    }

    auto estimator::estimate_pose_new
    (
        const frame::estimated& frame_0,
        const frame::tracked&   tracked_1
    ) const
        -> estimate_pose_result
    {
        if (tracked_1.index == 0)
        {
            return {};
        }

        pose_data pose{};
        try
        {
            pose = estimate_pose_3d2d(frame_0, tracked_1, 0);
        }
        catch (const std::exception& e0)
        {
            SPDLOG_WARN("estimate_pose_3d2d failed:");
            SPDLOG_WARN("{}", e0.what());
            SPDLOG_INFO("Trying 3D-2D pose estimation with camera 1");

            try
            {
                pose      = estimate_pose_3d2d(frame_0, tracked_1, 1);
                pose.pose = pose.pose * _calibration.cameras[1].pose_in_cam0.inv();
            }
            catch (const std::exception& e1)
            {
                SPDLOG_WARN("estimate_pose_3d2d with camera 1 also failed:");
                SPDLOG_WARN("{}", e1.what());
                SPDLOG_INFO("Trying 3D-3D pose estimation");

                // Estimate pose 3D-3D
                const auto& correspondences
                    = frame_0.points3d
                    | std::views::values
                    | std::views::filter([&tracked_1](const auto& point3d) { return tracked_1.points3d.contains_index(point3d.index); })
                    | std::views::transform([&tracked_1](const auto& point3d) { return std::make_pair(point3d, tracked_1.points3d.at(point3d.index)); })
                    | std::ranges::to<std::vector>();

                if (correspondences.size() < 3)
                {
                    throw std::logic_error("Not enough 3D-3D correspondences for rigid transform RANSAC: " + std::to_string(correspondences.size()) + " (need >= 3)");
                }

                const auto& points_0 = correspondences | std::views::transform([](const auto& pair) { return cv::Point3d{pair.first}; }) | std::ranges::to<std::vector>();
                const auto& points_1 = correspondences | std::views::transform([](const auto& pair) { return cv::Point3d{pair.second}; }) | std::ranges::to<std::vector>();

                cv::Matx33d         R;
                cv::Point3d         t;
                std::vector<size_t> inliers, outliers;
                std::vector<double> errors;

                auto success = utils::estimate_rigid_ransac(points_0, points_1, R, t, inliers, outliers, errors, _options.threshold_3d3d, 1000);

                if (!success)
                {
                    throw std::runtime_error("Rigid transform estimation (3D-3D) failed");
                }

                pose.pose     = cv::Affine3d{R, t};
                pose.inliers  = inliers | std::views::transform([&correspondences](const auto& idx) { return correspondences[idx].first.index; }) | std::ranges::to<std::vector>();
                pose.outliers = outliers | std::views::transform([&correspondences](const auto& idx) { return correspondences[idx].first.index; }) | std::ranges::to<std::vector>();
            }
        }

        return {.pose_3d2d = pose, .chosen_pose = pose.pose, .chosen_count = pose.inliers.size()};
    }

    auto estimator::estimate_pose_weighted
    (
        const estimate_pose_result& result
    )
        -> weighted_pose_result
    {
        return pose_estimation::pose_fusion::fuse_poses(result);
    }

    auto estimator::solvepnp_ransac
    (
        const std::vector<std::pair<point3d, keypoint>>& correspondences
    ) const
        -> pose_data
    {
        if (correspondences.size() < 4)
        {
            throw std::invalid_argument("cv::solvePnPRansac: needs at least 4 correspondences, but got " + std::to_string(correspondences.size()));
        }

        const auto object_points
            = correspondences
            | std::views::transform([](const auto& pair) { return cv::Point3f(pair.first.x, pair.first.y, pair.first.z); })
            | std::ranges::to<std::vector>();

        const auto image_points
            = correspondences
            | std::views::transform([](const auto& pair) { return pair.second.pt; })
            | std::ranges::to<std::vector>();

        auto R    = cv::Matx33d::eye();
        auto rvec = cv::Vec3d::zeros();
        auto tvec = cv::Vec3d::zeros();

        cv::Rodrigues(R, rvec);

        std::vector<int> inliers;

        const auto& success = cv::solvePnPRansac(object_points, image_points, _calibration.camera_matrix[0], {}, rvec, tvec, true, 100, _options.pnp.threshold, 0.99, inliers);

        if (!success)
        {
            throw std::runtime_error("cv::solvePnPRansac failed to find a valid pose");
        }

        if (inliers.size() < 4)
        {
            throw std::logic_error("cv::solvePnPRansac returned < 4 inliers, which is insufficient for a valid pose estimation");
        }

        cv::Rodrigues(rvec, R);

        pose_data pose{};
        pose.pose    = cv::Affine3d(R, tvec);
        pose.inliers = inliers | std::views::transform([](const auto& i) { return size_t{gsl::narrow<size_t>(i)}; }) | std::ranges::to<std::vector>();

        return pose;
    }

    auto estimator::solvepnp_refinelm
    (
        const std::vector<std::pair<point3d, keypoint>>& correspondences,
        const pose_data&                                 pose
    ) const
        -> pose_data
    {
        if (correspondences.size() < 4)
        {
            throw std::invalid_argument("cv::solvePnP: needs at least 4 correspondences, but got " + std::to_string(correspondences.size()));
        }

        const auto& object_points
            = correspondences
            | std::views::transform([](const auto& pair) { return cv::Point3f(pair.first.x, pair.first.y, pair.first.z); })
            | std::ranges::to<std::vector>();

        const auto& image_points
            = correspondences
            | std::views::transform([](const auto& pair) { return pair.second.pt; })
            | std::ranges::to<std::vector>();

        auto rvec = cv::Vec3d::zeros();
        cv::Vec3d tvec = {};
        cv::Rodrigues(pose.pose.rotation(), rvec);
        tvec = pose.pose.translation();

        // if (rvec.empty() || tvec.empty())
        // {
        //     throw std::runtime_error("Invalid initial pose for cv::solvePnPRefineLM: empty rotation or translation vector");
        // }

        cv::solvePnPRefineLM(object_points, image_points, _calibration.camera_matrix[0], {}, rvec, tvec, cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 20, 1e-6));

        cv::Matx33d R;
        cv::Rodrigues(rvec, R);

        return {cv::Affine3d(R, tvec), pose.indices, pose.inliers, pose.outliers};
    }
} // namespace zenslam
