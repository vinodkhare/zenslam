#include "zenslam/matching/matching_utils.h"

#include <ranges>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>

#include <spdlog/spdlog.h>

#include "zenslam/utils/utils.h"
#include "zenslam/utils/utils_opencv.h"
#include "zenslam/types/keyline.h"
#include "zenslam/types/keypoint.h"
#include "zenslam/types/point3d_cloud.h"

auto zenslam::utils::filter(
    const std::vector<cv::KeyPoint>& keypoints0,
    const std::vector<cv::KeyPoint>& keypoints1,
    const std::vector<cv::DMatch>&   matches,
    const cv::Matx33d&               fundamental,
    const double                     epipolar_threshold) -> std::vector<cv::DMatch>
{
    std::vector<cv::DMatch> filtered { };
    filtered.reserve(matches.size());

    if (auto [points0, points1] = to_points(keypoints0, keypoints1, matches);
        points0.size() == points1.size() && !points0.empty())
    {
        std::vector<cv::Vec3f> epilines0, epilines1;
        cv::computeCorrespondEpilines(points0, 1, fundamental, epilines1);
        cv::computeCorrespondEpilines(points1, 2, fundamental, epilines0);

        for (size_t i = 0; i < matches.size(); ++i)
        {
            const auto& pt0   = points0[i];
            const auto& pt1   = points1[i];
            const auto& line1 = epilines1[i];
            const auto& line0 = epilines0[i];

            const double err0 =
                std::abs(line0[0] * pt0.x + line0[1] * pt0.y + line0[2]) /
                std::sqrt(line0[0] * line0[0] + line0[1] * line0[1]);

            const double err1 =
                std::abs(line1[0] * pt1.x + line1[1] * pt1.y + line1[2]) /
                std::sqrt(line1[0] * line1[0] + line1[1] * line1[1]);

            if (err0 < epipolar_threshold && err1 < epipolar_threshold)
            {
                filtered.push_back(matches[i]);
            }
        }
    }
    else
    {
        filtered = matches;
    }

    return filtered;
}

auto zenslam::utils::create_matcher(
    const slam_options& options,
    const bool          is_binary) -> cv::Ptr<cv::DescriptorMatcher>
{
    const auto norm_type = is_binary ? cv::NORM_HAMMING : cv::NORM_L2;

    if (options.matcher == matcher_type::KNN)
    {
        // kNN matching with ratio test using BFMatcher
        return cv::makePtr<cv::BFMatcher>(norm_type, false);
    }
    else if (options.matcher == matcher_type::FLANN)
    {
        // FLANN-based matching with ratio test
        if (is_binary)
        {
            // LSH (Locality Sensitive Hashing) for binary descriptors
            return cv::makePtr<cv::FlannBasedMatcher>(
                cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));
        }
        else
        {
            // KDTree for float descriptors (SIFT, SURF, etc.)
            return cv::makePtr<cv::FlannBasedMatcher>(
                cv::makePtr<cv::flann::KDTreeIndexParams>(4));
        }
    }
    else // BRUTE
    {
        // Brute-force matching with cross-check
        return cv::makePtr<cv::BFMatcher>(norm_type, true);
    }
}

/**
 * Helper function to check if a 3D point projects within the camera frustum
 * @param point_camera 3D point in camera coordinates
 * @param projection Camera projection matrix
 * @param image_width Image width in pixels
 * @param image_height Image height in pixels
 * @param margin Margin in pixels (positive allows points slightly outside image bounds)
 * @return True if point is within frustum
 */
static auto is_in_frustum(
    const cv::Point3d& point_camera,
    const cv::Matx34d& projection,
    int                image_width,
    int                image_height,
    double             margin = 50.0) -> bool
{
    // Check if point is in front of camera
    if (point_camera.z <= 0.0)
        return false;

    // Project to image plane
    const cv::Mat point_homogeneous = (cv::Mat_<double>(4, 1) << point_camera.x, point_camera.y, point_camera.z, 1.0);
    const cv::Mat projected         = cv::Mat(projection) * point_homogeneous;

    if (std::abs(projected.at<double>(2)) < 1e-9)
        return false;

    const double u = projected.at<double>(0) / projected.at<double>(2);
    const double v = projected.at<double>(1) / projected.at<double>(2);

    // Check if within image bounds (with margin)
    return (u >= -margin && u < image_width + margin &&
        v >= -margin && v < image_height + margin);
}

auto zenslam::utils::match_keypoints3d(
    const point3d_cloud& points3d_world,
    const map<keypoint>& keypoints,
    const cv::Affine3d&  pose_of_camera0_in_world,
    const cv::Matx34d&   projection,
    const double         radius,
    const double         threshold) -> std::vector<cv::DMatch>
{
    if (points3d_world.empty() || keypoints.empty())
        return std::vector<cv::DMatch> { };

    // find keypoints that are not triangulated
    const auto& untriangulated = keypoints.values_unmatched(points3d_world) |
        std::ranges::to<std::vector>();

    if (untriangulated.empty())
        return std::vector<cv::DMatch> { };

    // Transform landmarks to camera frame and apply frustum culling
    // Note: For full frustum culling with image dimensions, use the overload that accepts slam_options
    const auto& points3d =
        pose_of_camera0_in_world.inv() * points3d_world.radius_search(
            static_cast<point3d>(pose_of_camera0_in_world.translation()),
            radius) |
        std::views::filter([](const auto& p3d) { return p3d.z > 0.0; }) |
        std::ranges::to<std::vector>();

    if (points3d.empty())
        return std::vector<cv::DMatch> { };

    cv::Mat descriptors3d = { };
    cv::vconcat(
        points3d | std::views::transform([](const auto& kp) { return kp.descriptor; }) |
        std::ranges::to<std::vector>(),

        descriptors3d // output
    );

    cv::Mat descriptors2d = { };
    cv::vconcat(
        untriangulated |
        std::views::transform([](const auto& keypoint) { return keypoint.descriptor; }) |
        std::ranges::to<std::vector>(),

        descriptors2d // output
    );

    if (descriptors3d.empty() || descriptors2d.empty())
        return std::vector<cv::DMatch> { };

    std::vector<cv::DMatch> matches_cv = { };
    cv::BFMatcher(
            descriptors3d.depth() == CV_8U ? cv::NORM_HAMMING : cv::NORM_L2,
            true)
       .match(descriptors3d, descriptors2d, matches_cv);

    // Apply reprojection filtering to remove outliers
    std::vector<cv::DMatch> matches = { };
    matches.reserve(matches_cv.size());

    // Project all 3D points to verify reprojection quality
    const auto& projected_all = project(to_points(points3d), projection);

    for (const auto& match : matches_cv)
    {
        const auto& point3d_idx  = match.queryIdx;
        const auto& keypoint_idx = match.trainIdx;
        const auto& point3d_cam  = points3d[point3d_idx];
        const auto& keypoint     = untriangulated[keypoint_idx];
        const auto& projected_pt = projected_all[point3d_idx];

        // Compute reprojection error
        const double error = cv::norm(projected_pt - cv::Point2d(keypoint.pt));

        // Filter by reprojection threshold
        if (error < threshold)
        {
            matches.emplace_back(
                point3d_cam.index,
                keypoint.index,
                static_cast<float>(error)); // Use reprojection error as distance metric
        }
    }

    return matches;
}

auto zenslam::utils::match_keypoints3d(
    const point3d_cloud& points3d_world,
    const map<keypoint>& keypoints,
    const cv::Affine3d&  pose_of_camera0_in_world,
    const cv::Matx34d&   projection,
    const cv::Size&      image_size,
    const double         radius,
    const slam_options&  options) -> std::vector<cv::DMatch>
{
    if (points3d_world.empty() || keypoints.empty())
        return std::vector<cv::DMatch> { };

    // find keypoints that are not triangulated
    const auto& untriangulated = keypoints.values_unmatched(points3d_world) | std::ranges::to<std::vector>();

    SPDLOG_DEBUG("  match_keypoints3d: total_world_points={}, untriangulated_kps={}",
                 points3d_world.size(), untriangulated.size());

    if (untriangulated.empty())
        return std::vector<cv::DMatch> { };

    // Transform landmarks to camera frame
    const auto points3d_camera =
        pose_of_camera0_in_world.inv() * points3d_world.radius_search(
            static_cast<point3d>(pose_of_camera0_in_world.translation()),
            radius);

    SPDLOG_DEBUG("  match_keypoints3d: after radius_search ({}m)={} points",
                 radius, points3d_camera.size());

    // Apply depth and frustum filtering
    std::vector<point3d> points3d;
    points3d.reserve(points3d_camera.size());

    for (const auto& p3d : points3d_camera)
    {
        // Check depth (must be in front of camera)
        if (p3d.z <= 0.0)
            continue;

        // Apply frustum culling if enabled
        if (options.enable_frustum_culling)
        {
            if (!is_in_frustum(
                cv::Point3d(p3d.x, p3d.y, p3d.z),
                projection,
                image_size.width,
                image_size.height,
                options.frustum_margin))
            {
                continue;
            }
        }

        points3d.emplace_back(p3d);
    }

    SPDLOG_DEBUG("  match_keypoints3d: after frustum/depth filter={} points",
                 points3d.size());

    if (points3d.empty())
        return std::vector<cv::DMatch> { };

    cv::Mat descriptors3d = { };
    cv::vconcat(
        points3d | std::views::transform([](const auto& kp) { return kp.descriptor; }) |
        std::ranges::to<std::vector>(),
        descriptors3d // output
    );

    cv::Mat descriptors2d = { };
    cv::vconcat(
        untriangulated |
        std::views::transform([](const auto& keypoint) { return keypoint.descriptor; }) |
        std::ranges::to<std::vector>(),
        descriptors2d // output
    );

    if (descriptors3d.empty() || descriptors2d.empty())
        return std::vector<cv::DMatch> { };

    std::vector<cv::DMatch> matches_cv = { };
    cv::BFMatcher(
            descriptors3d.depth() == CV_8U ? cv::NORM_HAMMING : cv::NORM_L2,
            true)
       .match(descriptors3d, descriptors2d, matches_cv);

    // Apply reprojection filtering to remove outliers
    std::vector<cv::DMatch> matches = { };
    matches.reserve(matches_cv.size());

    // Project all 3D points to verify reprojection quality
    const auto& projected_all = project(to_points(points3d), projection);

    for (const auto& match : matches_cv)
    {
        const auto& point3d_idx  = match.queryIdx;
        const auto& keypoint_idx = match.trainIdx;
        const auto& point3d_cam  = points3d[point3d_idx];
        const auto& keypoint     = untriangulated[keypoint_idx];
        const auto& projected_pt = projected_all[point3d_idx];

        // Compute reprojection error
        const double error = cv::norm(projected_pt - cv::Point2d(keypoint.pt));

        // Filter by configured reprojection threshold
        if (error < options.reprojection_threshold_3d2d)
        {
            matches.emplace_back(
                point3d_cam.index,
                keypoint.index,
                static_cast<float>(error)); // Use reprojection error as distance metric
        }
    }

    SPDLOG_DEBUG(
        "3D-2D matching: {} landmarks in radius, {} after frustum culling, {} descriptor matches, {} after reprojection filtering (threshold={:.1f}px)",
        points3d_camera.size(),
        points3d.size(),
        matches_cv.size(),
        matches.size(),
        options.reprojection_threshold_3d2d);

    return matches;
}

auto zenslam::utils::match_keylines(
    const map<keyline>& keylines_map_0,
    const map<keyline>& keylines_map_1,
    const cv::Matx33d&  fundamental,
    double              epipolar_threshold) -> std::vector<cv::DMatch>
{
    // Prepare descriptors and keyline vectors
    std::vector<keyline> keylines_0,    keylines_1;
    cv::Mat              descriptors_0, descriptors_1;

    for (const auto& kl : keylines_map_0 | std::views::values)
    {
        keylines_0.push_back(kl);
        if (descriptors_0.empty())
            descriptors_0 = kl.descriptor;
        else
            cv::vconcat(descriptors_0, kl.descriptor, descriptors_0);
    }

    for (const auto& kl : keylines_map_1 | std::views::values)
    {
        keylines_1.push_back(kl);
        if (descriptors_1.empty())
            descriptors_1 = kl.descriptor;
        else
            cv::vconcat(descriptors_1, kl.descriptor, descriptors_1);
    }

    // Match descriptors
    const cv::BFMatcher     matcher(cv::NORM_HAMMING, true);
    std::vector<cv::DMatch> matches;
    if (!descriptors_0.empty() && !descriptors_1.empty())
        matcher.match(descriptors_0, descriptors_1, matches);

    // Epipolar filtering on endpoints and midpoint
    std::vector<cv::DMatch> filtered;
    filtered.reserve(matches.size());

    for (const auto& match : matches)
    {
        auto kl0 = keylines_0[match.queryIdx];
        auto kl1 = keylines_1[match.trainIdx];

        // Endpoints and midpoint
        std::vector pts0 = {
            cv::Point2f(kl0.startPointX, kl0.startPointY),
            cv::Point2f(kl0.endPointX, kl0.endPointY),
            cv::Point2f(kl0.pt.x, kl0.pt.y)
        };

        std::vector pts1 = {
            cv::Point2f(kl1.startPointX, kl1.startPointY),
            cv::Point2f(kl1.endPointX, kl1.endPointY),
            cv::Point2f(kl1.pt.x, kl1.pt.y)
        };

        // Compute epilines for pts0 in image 1, and pts1 in image 0
        std::vector<cv::Vec3f> epilines0, epilines1;
        cv::computeCorrespondEpilines(pts0, 1, fundamental, epilines1);
        cv::computeCorrespondEpilines(pts1, 2, fundamental, epilines0);

        auto good = true;
        for (auto i = 0; i < 3; ++i)
        {
            // Error for pt0 to epiline in image 0
            double err0 = std::abs(
                    epilines0[i][0] * pts0[i].x +
                    epilines0[i][1] * pts0[i].y + epilines0[i][2]) /
                std::sqrt(
                    epilines0[i][0] * epilines0[i][0] +
                    epilines0[i][1] * epilines0[i][1]);

            // Error for pt1 to epiline in image 1
            double err1 = std::abs(
                    epilines1[i][0] * pts1[i].x +
                    epilines1[i][1] * pts1[i].y + epilines1[i][2]) /
                std::sqrt(
                    epilines1[i][0] * epilines1[i][0] +
                    epilines1[i][1] * epilines1[i][1]);

            if (err0 > epipolar_threshold || err1 > epipolar_threshold)
            {
                good = false;
                break;
            }
        }
        if (good)
        {
            // Use keyline indices for DMatch
            filtered.emplace_back(kl0.index, kl1.index, match.distance);
        }
    }

    return filtered;
}

auto zenslam::utils::match_temporal(
    const std::map<size_t, keypoint>& keypoints_map_0,
    const std::map<size_t, keypoint>& keypoints_map_1,
    const cv::Matx33d&                camera_matrix,
    double                            threshold) -> std::vector<cv::DMatch>
{
    cv::Mat                 descriptors_0 { };
    cv::Mat                 descriptors_1 { };
    std::vector<keypoint>   unmatched_0 { };
    std::vector<keypoint>   unmatched_1 { };
    std::vector<cv::DMatch> matches_new { };

    for (const auto& keypoint_0 : keypoints_map_0 | std::views::values)
    {
        if (keypoints_map_1.contains(keypoint_0.index))
            continue;

        unmatched_0.emplace_back(keypoint_0);

        if (descriptors_0.empty())
        {
            descriptors_0 = keypoint_0.descriptor;
        }
        else
        {
            cv::vconcat(descriptors_0, keypoint_0.descriptor, descriptors_0);
        }
    }

    for (const auto& keypoint_1 : keypoints_map_1 | std::views::values)
    {
        if (keypoints_map_0.contains(keypoint_1.index))
            continue;

        unmatched_1.emplace_back(keypoint_1);

        if (descriptors_1.empty())
        {
            descriptors_1 = keypoint_1.descriptor;
        }
        else
        {
            cv::vconcat(descriptors_1, keypoint_1.descriptor, descriptors_1);
        }
    }

    if (unmatched_0.size() < 5 || unmatched_1.size() < 5)
    {
        return matches_new;
    }

    const cv::BFMatcher matcher {
        descriptors_0.depth() == CV_8U ? cv::NORM_HAMMING : cv::NORM_L2,
        true
    };
    std::vector<cv::DMatch> matches;
    matcher.match(descriptors_0, descriptors_1, matches);

    // Filter matches based on reprojection error
    auto points_0 = matches |
        std::views::transform(
            [&unmatched_0](const auto& match)
            {
                return unmatched_0[match.queryIdx].pt;
            }) |
        std::ranges::to<std::vector>();

    auto points_1 = matches |
        std::views::transform(
            [&unmatched_1](const auto& match)
            {
                return unmatched_1[match.trainIdx].pt;
            }) |
        std::ranges::to<std::vector>();

    auto mask = std::vector<uchar> { };

    auto e = cv::findEssentialMat(
        points_0,
        points_1,
        camera_matrix,
        cv::RANSAC,
        0.99,
        threshold,
        mask);

    auto E = cv::Matx33d(
        e.at<float>(0, 0),
        e.at<float>(0, 1),
        e.at<float>(0, 2),
        e.at<float>(1, 0),
        e.at<float>(1, 1),
        e.at<float>(1, 2),
        e.at<float>(2, 0),
        e.at<float>(2, 1),
        e.at<float>(2, 2));

    for (const auto& [match, msk] : std::ranges::views::zip(matches, mask))
    {
        if (msk)
        {
            const auto& keypoint_l = unmatched_0[match.queryIdx];
            const auto& keypoint_r = unmatched_1[match.trainIdx];

            const auto& pt_l  = cv::Vec3d { keypoint_l.pt.x, keypoint_l.pt.y, 1.0f };
            const auto& pt_r  = cv::Vec3d { keypoint_r.pt.x, keypoint_r.pt.y, 1.0f };
            const auto& error =
                pt_r.t() * camera_matrix.inv().t() * E * camera_matrix.inv() * pt_l;

            if (error[0] > threshold || match.distance > 5)
            {
                continue;
            }

            matches_new.emplace_back(
                keypoint_l.index,
                keypoint_r.index,
                match.distance);
        }
    }

    return matches_new;
}

auto zenslam::utils::solve_pnp(
    const cv::Matx33d&              camera_matrix,
    const std::vector<cv::Point3d>& points3d,
    const std::vector<cv::Point2d>& points2d,
    cv::Affine3d&                   pose) -> void
{
    cv::Mat          rvec { pose.rvec() };
    cv::Mat          tvec { pose.translation() };
    std::vector<int> inliers { };

    SPDLOG_INFO("SolvePnP with {} points", points3d.size());

    if (cv::solvePnPRansac(
        points3d,
        points2d,
        camera_matrix,
        cv::Mat(),
        rvec,
        tvec,
        true,
        1000,
        4.0,
        0.99,
        inliers))
    {
        SPDLOG_INFO(
            "SolvePnP successful with {} inliers out of {} points",
            inliers.size(),
            points3d.size());
        pose = cv::Affine3d(rvec, tvec);
    }
    else
    {
        SPDLOG_WARN("SolvePnP failed");
        throw std::runtime_error("SolvePnP failed");
    }
}
