#include "zenslam/estimator.h"

#include <set>

#include <gsl/narrow>

#include <opencv2/calib3d.hpp>

#include <spdlog/spdlog.h>

#include "zenslam/utils_slam.h"

namespace zenslam
{
    estimator::estimator(const calibration& calib, const class options::slam& opts)
        : _calibration(calib), _options(opts)
    {
    }

    auto estimator::estimate_pose_3d2d
    (
        const std::map<size_t, point3d>&  map_points_0,
        const std::map<size_t, keypoint>& map_keypoints_1
    ) const -> pose_data
    {
        std::vector<cv::Point3d> points3d;
        std::vector<cv::Point2d> points2d;
        std::vector<size_t>      indices;
        utils::correspondences_3d2d(map_points_0, map_keypoints_1, points3d, points2d, indices);

        cv::Affine3d pose = cv::Affine3d::Identity();
        auto         out  = zenslam::pose_data { };

        if (points3d.size() >= 6)
        {
            cv::Mat          rvec { pose.rvec() };
            cv::Mat          tvec { pose.translation() };
            std::vector<int> inliers;
            if (cv::solvePnPRansac
                (
                    points3d,
                    points2d,
                    _calibration.camera_matrix[0],
                    cv::Mat(),
                    rvec,
                    tvec,
                    true,
                    1000,
                    gsl::narrow<float>(_options.threshold_3d2d),
                    0.99,
                    inliers
                ))
            {
                out.pose    = cv::Affine3d(rvec, tvec);
                out.indices = indices;

                for (auto i : inliers)
                    out.inliers.push_back(indices[i]);
                auto inliers_set = std::set(out.inliers.begin(), out.inliers.end());
                for (auto i : indices)
                    if (!inliers_set.contains(i))
                        out.outliers.push_back(i);

                std::vector<cv::Point2d> points2d_back;
                cv::projectPoints(points3d, rvec, tvec, _calibration.camera_matrix[0], cv::Mat(), points2d_back);

                for (auto i : inliers)
                {
                    const auto& p = points2d[i];
                    const auto& q = points2d_back[i];
                    out.errors.push_back(cv::norm(p - q));
                }

                return out;
            }

            throw std::runtime_error("SolvePnP failed");
        }

        throw std::runtime_error("Not enough 3D-2D correspondences to compute pose (need > 6)");
    }

    auto estimator::estimate_pose_3d3d
    (
        const std::map<size_t, point3d>& map_points_0,
        const std::map<size_t, point3d>& map_points_1
    ) const -> pose_data
    {
        std::vector<cv::Point3d> points_0;
        std::vector<cv::Point3d> points_1;
        std::vector<size_t>      indices;
        utils::correspondences_3d3d(map_points_0, map_points_1, points_0, points_1, indices);

        if (points_0.size() >= 3)
        {
            cv::Matx33d         R;
            cv::Point3d         t;
            std::vector<size_t> inliers;
            std::vector<size_t> outliers;
            std::vector<double> errors;
            utils::estimate_rigid_ransac(points_0, points_1, R, t, inliers, outliers, errors, _options.threshold_3d3d, 1000);
            return { cv::Affine3d { R, t }, indices, inliers, outliers, errors };
        }

        throw std::runtime_error("Not enough 3D-3D correspondences to compute pose (need > 3)");
    }

    auto estimator::estimate_pose(const std::map<size_t, point3d>& points3d_0, const frame::tracked& tracked_1) const -> estimate_pose_result
    {
        estimate_pose_result result { };

        // Attempt 3D-3D
        try
        {
            result.pose_3d3d = estimate_pose_3d3d(points3d_0, tracked_1.points3d);
        }
        catch (const std::runtime_error& e)
        {
            SPDLOG_WARN("3D-3D pose estimation failed: {}", e.what());
        }

        // Attempt 3D-2D (left camera)
        try
        {
            result.pose_3d2d = estimate_pose_3d2d(points3d_0, tracked_1.keypoints[0]);
        }
        catch (const std::runtime_error& e)
        {
            SPDLOG_WARN("3D-2D pose estimation failed: {}", e.what());
        }

        // Choose the pose with more inliers; if tie, prefer 3D-3D; if both invalid, identity
        const auto inliers_3d3d = result.pose_3d3d.inliers.size();
        const auto inliers_3d2d = result.pose_3d2d.inliers.size();

        if (inliers_3d3d == 0 && inliers_3d2d == 0)
            result.chosen_pose = cv::Affine3d::Identity();
        else if (inliers_3d3d >= inliers_3d2d)
            result.chosen_pose = result.pose_3d3d.pose;
        else
            result.chosen_pose = result.pose_3d2d.pose;

        return result;
    }
}
