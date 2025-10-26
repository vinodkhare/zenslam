#include "zenslam/utils_slam.h"

#include <map>
#include <numeric>
#include <random>
#include <tuple>
#include <vector>

#include <gsl/narrow>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/video/tracking.hpp>

#include <spdlog/spdlog.h>

#include "zenslam/calibration.h"
#include "zenslam/grid_detector.h"
#include "zenslam/pose_data.h"
#include "zenslam/utils.h"
#include "zenslam/utils_opencv.h"
#include "zenslam/frame/processed.h"
#include "zenslam/types/point3d.h"
#include "zenslam/types/point3d_cloud.h"
#include "zenslam/preint.h"

#if ZENSLAM_HAS_UGPM
#include <preint/preint.h>
#endif

void zenslam::utils::correspondences_3d2d
(
    const std::map<size_t, point3d>&  points,
    const std::map<size_t, keypoint>& keypoints,
    std::vector<cv::Point3d>&         points3d,
    std::vector<cv::Point2d>&         points2d,
    std::vector<size_t>&              indices
)
{
    for (const auto& index : keypoints | std::views::keys)
    {
        if (points.contains(index))
        {
            points3d.emplace_back(points.at(index));
            points2d.emplace_back(keypoints.at(index).pt);
            indices.emplace_back(index);
        }
    }
}

void zenslam::utils::correspondences_3d3d
(
    const std::map<size_t, point3d>& points_map_0,
    const std::map<size_t, point3d>& points_map_1,
    std::vector<cv::Point3d>&        points3d_0,
    std::vector<cv::Point3d>&        points3d_1,
    std::vector<size_t>&             indexes
)
{
    for (const auto& index : points_map_1 | std::views::keys)
    {
        if (points_map_0.contains(index))
        {
            points3d_0.emplace_back(points_map_0.at(index));
            points3d_1.emplace_back(points_map_1.at(index));
            indexes.emplace_back(index);
        }
    }
}

auto zenslam::utils::estimate_pose_3d2d
(
    const std::map<size_t, point3d>&  map_points_0,
    const std::map<size_t, keypoint>& map_keypoints_1,
    const cv::Matx33d&                camera_matrix,
    const double&                     threshold
) -> pose_data
{
    std::vector<cv::Point3d> points3d;
    std::vector<cv::Point2d> points2d;
    std::vector<size_t>      indices = { };
    correspondences_3d2d(map_points_0, map_keypoints_1, points3d, points2d, indices);

    cv::Affine3d pose      = { cv::Affine3d::Identity() };
    auto         pose_data = zenslam::pose_data { };

    if (points3d.size() >= 6)
    {
        cv::Mat          rvec { pose.rvec() };
        cv::Mat          tvec { pose.translation() };
        std::vector<int> inliers { };
        if (cv::solvePnPRansac
            (
                points3d,
                points2d,
                camera_matrix,
                cv::Mat(),
                rvec,
                tvec,
                true,
                1000,
                gsl::narrow<float>(threshold),
                0.99,
                inliers
            ))
        {
            pose_data.pose    = cv::Affine3d(rvec, tvec);
            pose_data.indices = indices;

            for (auto i : inliers)
                pose_data.inliers.push_back(indices[i]);

            auto inliers_set = std::set(pose_data.inliers.begin(), pose_data.inliers.end());
            for (auto i : indices)
                if (!inliers_set.contains(i))
                    pose_data.outliers.push_back(i);

            auto points2d_back = std::vector<cv::Point2d> { };
            cv::projectPoints(points3d, rvec, tvec, camera_matrix, cv::Mat(), points2d_back);

            for (auto i : inliers)
            {
                const auto& p = points2d[i];
                const auto& q = points2d_back[i];
                pose_data.errors.push_back(cv::norm(p - q));
            }

            return pose_data;
        }

        throw std::runtime_error("SolvePnP failed");
    }

    throw std::runtime_error("Not enough 3D-2D correspondences to compute pose (need > 6)");
}

auto zenslam::utils::estimate_pose_3d3d
(
    const std::map<size_t, point3d>& map_points_0,
    const std::map<size_t, point3d>& map_points_1,
    const double&                    threshold
) -> pose_data
{
    // Gather points from slam.frame[0] and slam.frame[1] for 3D-3D pose computation
    std::vector<cv::Point3d> points_0 { };
    std::vector<cv::Point3d> points_1 { };
    std::vector<size_t>      indices { };
    correspondences_3d3d(map_points_0, map_points_1, points_0, points_1, indices);

    // Compute relative pose between slam.frame[0] and slam.frame[1] using 3D-3D correspondences
    if (points_0.size() >= 3)
    {
        cv::Matx33d         R;
        cv::Point3d         t;
        std::vector<size_t> inliers { };
        std::vector<size_t> outliers { };
        std::vector<double> errors { };
        estimate_rigid_ransac(points_0, points_1, R, t, inliers, outliers, errors, threshold, 1000);

        return { cv::Affine3d { R, t }, indices, inliers, outliers, errors };
    }

    throw std::runtime_error("Not enough 3D-3D correspondences to compute pose (need > 3)");
}

bool zenslam::utils::estimate_rigid
(
    const std::vector<cv::Point3d>& points3d_0,
    const std::vector<cv::Point3d>& points3d_1,
    cv::Matx33d&                    R,
    cv::Point3d&                    t
)
{
    if (points3d_0.size() != points3d_1.size() || points3d_0.size() < 3)
        return false;

    // Compute means
    auto mean_src = std::accumulate
        (
            points3d_0.begin(),
            points3d_0.end(),
            cv::Vec3d(0, 0, 0),
            [](const cv::Vec3d& acc, const cv::Point3d& p)
            {
                return acc + cv::Vec3d(p.x, p.y, p.z);
            }
        ) /
        gsl::narrow<double>(points3d_0.size());

    auto mean_dst = std::accumulate
        (
            points3d_1.begin(),
            points3d_1.end(),
            cv::Vec3d(0, 0, 0),
            [](const cv::Vec3d& acc, const cv::Point3d& p)
            {
                return acc + cv::Vec3d(p.x, p.y, p.z);
            }
        ) /
        gsl::narrow<double>(points3d_1.size());

    // Compute cross-covariance
    cv::Matx33d Sigma(0, 0, 0, 0, 0, 0, 0, 0, 0);
    for (size_t i = 0; i < points3d_0.size(); ++i)
    {
        auto a = cv::Vec3d(points3d_0[i].x, points3d_0[i].y, points3d_0[i].z) - mean_src;
        auto b = cv::Vec3d(points3d_1[i].x, points3d_1[i].y, points3d_1[i].z) - mean_dst;
        Sigma(0, 0) += b[0] * a[0];
        Sigma(0, 1) += b[0] * a[1];
        Sigma(0, 2) += b[0] * a[2];
        Sigma(1, 0) += b[1] * a[0];
        Sigma(1, 1) += b[1] * a[1];
        Sigma(1, 2) += b[1] * a[2];
        Sigma(2, 0) += b[2] * a[0];
        Sigma(2, 1) += b[2] * a[1];
        Sigma(2, 2) += b[2] * a[2];
    }
    Sigma *= 1.0 / gsl::narrow<double>(points3d_0.size());

    // SVD
    cv::Mat Sigma_mat(3, 3, CV_64F);
    for (auto r = 0; r < 3; ++r)
        for (auto c                    = 0; c < 3; ++c)
            Sigma_mat.at<double>(r, c) = Sigma(r, c);
    cv::Mat U, S, Vt;
    cv::SVD::compute(Sigma_mat, S, U, Vt);
    cv::Mat R_mat = U * Vt;
    if (cv::determinant(R_mat) < 0)
    {
        cv::Mat Sfix          = cv::Mat::eye(3, 3, CV_64F);
        Sfix.at<double>(2, 2) = -1.0;
        R_mat                 = U * Sfix * Vt;
    }
    R = cv::Matx33d(R_mat);
    t = mean_dst - cv::Vec3d(R * mean_src);
    return true;
}

auto zenslam::utils::predict_pose_from_imu
(
    const cv::Affine3d&     pose_0,
    const cv::Vec3d&        velocity_0,
    const frame::processed& frame,
    const cv::Vec3d&        gravity
) -> cv::Affine3d
{
#if ZENSLAM_HAS_UGPM
    return preint::predict_pose(pose_0, velocity_0, frame.preint, gravity);
#else
    // When ugpm is not available, return the previous pose (constant velocity model)
    SPDLOG_WARN("IMU preintegration not available, using identity motion model");
    return pose_0;
#endif
}

auto zenslam::utils::estimate_rigid_ransac
(
    const std::vector<cv::Point3d>& src,
    const std::vector<cv::Point3d>& dst,
    cv::Matx33d&                    best_R,
    cv::Point3d&                    best_t,
    std::vector<size_t>&            inlier_indices,
    std::vector<size_t>&            outlier_indices,
    std::vector<double>&            errors,
    const double                    threshold,
    const int                       max_iterations
) -> bool
{
    if (src.size() != dst.size() || src.size() < 3)
        return false;

    std::mt19937                          rng { std::random_device { }() };
    std::uniform_int_distribution<size_t> dist(0, src.size() - 1);

    auto                best_inlier_count = 0;
    std::vector<size_t> best_inliers;
    cv::Matx33d         bestR;
    cv::Vec3d           bestt;
    std::vector<double> best_errors { };

    for (auto iter = 0; iter < max_iterations; ++iter)
    {
        // Randomly sample 3 unique indices
        std::set<size_t> idx_set;
        while (idx_set.size() < 3)
            idx_set.insert(dist(rng));
        std::vector indices(idx_set.begin(), idx_set.end());

        // Build minimal sets
        std::vector<cv::Point3d> src_sample, dst_sample;
        for (const auto i : indices)
        {
            src_sample.push_back(src[i]);
            dst_sample.push_back(dst[i]);
        }

        cv::Matx33d R;
        cv::Point3d t;
        if (!estimate_rigid(src_sample, dst_sample, R, t))
            continue;

        // Count inliers
        std::vector<size_t> inliers;
        std::vector<double> errs;
        for (size_t i = 0; i < src.size(); ++i)
        {
            cv::Vec3d  p = R * cv::Point3d(src[i].x, src[i].y, src[i].z) + t;
            cv::Vec3d  q(dst[i].x, dst[i].y, dst[i].z);
            const auto err = cv::norm(p - q);

            if (err < threshold)
            {
                inliers.push_back(i);
                errs.push_back(err);
            }
        }

        if (inliers.size() > best_inlier_count)
        {
            best_inlier_count = static_cast<int>(inliers.size());
            bestR             = R;
            bestt             = t;
            best_inliers      = inliers;
            best_errors       = errs;
        }
    }

    outlier_indices.clear();
    if (best_inlier_count >= 3)
    {
        best_R         = bestR;
        best_t         = bestt;
        errors         = best_errors;
        inlier_indices = best_inliers;

        for (size_t i = 0; i < src.size(); ++i)
        {
            if (std::ranges::find(inlier_indices, i) == inlier_indices.end())
                outlier_indices.push_back(i);
        }

        return true;
    }
    return false;
}

auto zenslam::utils::filter
(
    const std::vector<cv::KeyPoint>& keypoints0,
    const std::vector<cv::KeyPoint>& keypoints1,
    const std::vector<cv::DMatch>&   matches,
    const cv::Matx33d&               fundamental,
    const double                     epipolar_threshold
) -> std::vector<cv::DMatch>
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

            const double err0 = std::abs(line0[0] * pt0.x + line0[1] * pt0.y + line0[2]) /
                std::sqrt(line0[0] * line0[0] + line0[1] * line0[1]);

            const double err1 = std::abs(line1[0] * pt1.x + line1[1] * pt1.y + line1[2]) /
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

auto zenslam::utils::create_matcher
(
    const class options::slam& options,
    bool                       is_binary
) -> cv::Ptr<cv::DescriptorMatcher>
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
            return cv::makePtr<cv::FlannBasedMatcher>
            (
                cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2)
            );
        }
        else
        {
            // KDTree for float descriptors (SIFT, SURF, etc.)
            return cv::makePtr<cv::FlannBasedMatcher>
            (
                cv::makePtr<cv::flann::KDTreeIndexParams>(4)
            );
        }
    }
    else // BRUTE
    {
        // Brute-force matching with cross-check
        return cv::makePtr<cv::BFMatcher>(norm_type, true);
    }
}

auto zenslam::utils::match_keypoints
(
    const map<keypoint>&                  keypoints_0,
    const map<keypoint>&                  keypoints_1,
    const cv::Ptr<cv::DescriptorMatcher>& matcher,
    const class options::slam&            options
) -> std::vector<cv::DMatch>
{
    cv::Mat                 descriptors_l { };
    cv::Mat                 descriptors_r { };
    std::vector<keypoint>   unmatched_l { };
    std::vector<keypoint>   unmatched_r { };
    std::vector<cv::DMatch> matches_new { };

    for (const auto& keypoint_l : keypoints_0 | std::views::values)
    {
        if (keypoints_1.contains(keypoint_l.index))
            continue;

        unmatched_l.emplace_back(keypoint_l);

        if (descriptors_l.empty())
        {
            descriptors_l = keypoint_l.descriptor;
        }
        else
        {
            cv::vconcat(descriptors_l, keypoint_l.descriptor, descriptors_l);
        }
    }

    for (const auto& keypoint_r : keypoints_1 | std::views::values)
    {
        if (keypoints_0.contains(keypoint_r.index))
            continue;

        unmatched_r.emplace_back(keypoint_r);

        if (descriptors_r.empty())
        {
            descriptors_r = keypoint_r.descriptor;
        }
        else
        {
            cv::vconcat(descriptors_r, keypoint_r.descriptor, descriptors_r);
        }
    }

    if (descriptors_l.empty() || descriptors_r.empty())
        return matches_new;

    std::vector<cv::DMatch> matches = { };
    const bool use_ratio_test = options.matcher == matcher_type::KNN || options.matcher == matcher_type::FLANN;

    if (use_ratio_test)
    {
        // kNN matching with ratio test (for KNN and FLANN modes)
        std::vector<std::vector<cv::DMatch>> knn_matches;
        matcher->knnMatch(descriptors_l, descriptors_r, knn_matches, 2);

        // Apply Lowe's ratio test
        for (const auto& knn : knn_matches)
        {
            if (knn.size() == 2 && knn[0].distance < options.matcher_ratio * knn[1].distance)
            {
                matches.push_back(knn[0]);
            }
        }
    }
    else
    {
        // Brute-force matching with cross-check (1-NN)
        matcher->match(descriptors_l, descriptors_r, matches);
    }

    // Estimate fundamental matrix using RANSAC and filter matches
    if (matches.size() >= 8)
    {
        auto points_l = matches | std::views::transform
        (
            [&unmatched_l](const auto& match)
            {
                return unmatched_l[match.queryIdx].pt;
            }
        ) | std::ranges::to<std::vector>();

        auto points_r = matches | std::views::transform
        (
            [&unmatched_r](const auto& match)
            {
                return unmatched_r[match.trainIdx].pt;
            }
        ) | std::ranges::to<std::vector>();

        std::vector<uchar> inlier_mask;
        cv::Mat            fundamental = cv::findFundamentalMat
        (
            points_l,
            points_r,
            cv::FM_RANSAC,
            options.epipolar_threshold,
            0.99,
            inlier_mask
        );

        // Keep only inliers
        std::vector<cv::DMatch> filtered_matches;
        for (size_t i = 0; i < matches.size(); ++i)
        {
            if (inlier_mask[i])
            {
                filtered_matches.push_back(matches[i]);
            }
        }

        matches = filtered_matches;
    }

    for (const auto& match : matches)
    {
        const auto& keypoint_l = unmatched_l[match.queryIdx];
        const auto& keypoint_r = unmatched_r[match.trainIdx];

        matches_new.emplace_back(keypoint_l.index, keypoint_r.index, match.distance);
    }

    return matches_new;
}

auto zenslam::utils::match_keypoints3d
(
    const point3d_cloud& points3d_world,
    const map<keypoint>& keypoints,
    const cv::Affine3d&  pose_of_camera0_in_world,
    const cv::Matx34d&   projection,
    const double         radius,
    const double         threshold
) -> std::vector<cv::DMatch>
{
    if (points3d_world.empty() || keypoints.empty())
        return std::vector<cv::DMatch> { };

    // find keypoints that are not triangulated
    const auto& untriangulated = keypoints.values_unmatched(points3d_world) | std::ranges::to<std::vector>();

    if (untriangulated.empty())
        return std::vector<cv::DMatch> { };

    const auto& points3d = pose_of_camera0_in_world.inv() * points3d_world.radius_search
        (
            static_cast<point3d>(pose_of_camera0_in_world.translation()),
            radius
        )
        | std::views::filter // TODO: add frustum filtering
        (
            [](const auto& p3d)
            {
                return p3d.z > 0.0;
            }
        )
        | std::ranges::to<std::vector>();

    if (points3d.empty())
        return std::vector<cv::DMatch> { };

    cv::Mat descriptors3d = { };
    cv::vconcat
    (
        points3d
        | std::views::transform
        (
            [](const auto& kp)
            {
                return kp.descriptor;
            }
        )
        | std::ranges::to<std::vector>(),

        descriptors3d // output
    );

    cv::Mat descriptors2d = { };
    cv::vconcat
    (
        untriangulated
        | std::views::transform
        (
            [](const auto& keypoint)
            {
                return keypoint.descriptor;
            }
        )
        | std::ranges::to<std::vector>(),

        descriptors2d // output
    );

    if (descriptors3d.empty() || descriptors2d.empty())
        return std::vector<cv::DMatch> { };

    std::vector<cv::DMatch> matches_cv = { };
    cv::BFMatcher
    (
        descriptors3d.depth() == CV_8U ? cv::NORM_HAMMING : cv::NORM_L2,
        true
    ).match(descriptors3d, descriptors2d, matches_cv);

    // TODO: add reprojection filtering
    std::vector<cv::DMatch> matches           = { };
    std::vector<point3d>    matched_points3d  = { };
    std::vector<keypoint>   matched_keypoints = { };

    for (const auto match : matches_cv)
    {
        matched_points3d.emplace_back(points3d[match.queryIdx]);
        matched_keypoints.emplace_back(untriangulated[match.trainIdx]);
    }

    const auto& projected = project(to_points(points3d), projection);
    const auto& errors    = vecnorm(projected - to_points(matched_keypoints));

    for (auto i = 0; i < matches_cv.size(); ++i)
    {
        if (errors[i] < threshold) // TODO: parameterize
        {
            matches.emplace_back
            (
                matched_points3d[i].index,
                matched_keypoints[i].index,
                matches_cv[i].distance
            );
        }
    }

    return matches;
}

auto zenslam::utils::match_keylines
(
    const map<keyline>& keylines_map_0,
    const map<keyline>& keylines_map_1,
    const cv::Matx33d&  fundamental,
    double              epipolar_threshold
) -> std::vector<cv::DMatch>
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
        std::vector pts0 =
        {
            cv::Point2f(kl0.startPointX, kl0.startPointY),
            cv::Point2f(kl0.endPointX, kl0.endPointY),
            cv::Point2f(kl0.pt.x, kl0.pt.y)
        };

        std::vector pts1 =
        {
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
            double err0 = std::abs(epilines0[i][0] * pts0[i].x + epilines0[i][1] * pts0[i].y + epilines0[i][2]) /
                std::sqrt(epilines0[i][0] * epilines0[i][0] + epilines0[i][1] * epilines0[i][1]);

            // Error for pt1 to epiline in image 1
            double err1 = std::abs(epilines1[i][0] * pts1[i].x + epilines1[i][1] * pts1[i].y + epilines1[i][2]) /
                std::sqrt(epilines1[i][0] * epilines1[i][0] + epilines1[i][1] * epilines1[i][1]);

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

auto zenslam::utils::match_temporal
(
    const std::map<size_t, keypoint>& keypoints_map_0,
    const std::map<size_t, keypoint>& keypoints_map_1,
    const cv::Matx33d&                camera_matrix,
    double                            threshold
) -> std::vector<cv::DMatch>
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

    const cv::BFMatcher     matcher { descriptors_0.depth() == CV_8U ? cv::NORM_HAMMING : cv::NORM_L2, true };
    std::vector<cv::DMatch> matches;
    matcher.match(descriptors_0, descriptors_1, matches);

    // Filter matches based on reprojection error
    auto points_0 = matches | std::views::transform
    (
        [&unmatched_0](const auto& match)
        {
            return unmatched_0[match.queryIdx].pt;
        }
    ) | std::ranges::to<std::vector>();

    auto points_1 = matches | std::views::transform
    (
        [&unmatched_1](const auto& match)
        {
            return unmatched_1[match.trainIdx].pt;
        }
    ) | std::ranges::to<std::vector>();

    auto mask = std::vector<uchar> { };

    auto e = cv::findEssentialMat(points_0, points_1, camera_matrix, cv::RANSAC, 0.99, threshold, mask);

    auto E = cv::Matx33d
    (
        e.at<float>(0, 0),
        e.at<float>(0, 1),
        e.at<float>(0, 2),
        e.at<float>(1, 0),
        e.at<float>(1, 1),
        e.at<float>(1, 2),
        e.at<float>(2, 0),
        e.at<float>(2, 1),
        e.at<float>(2, 2)
    );

    for (const auto& [match, msk] : std::ranges::views::zip(matches, mask))
    {
        if (msk)
        {
            const auto& keypoint_l = unmatched_0[match.queryIdx];
            const auto& keypoint_r = unmatched_1[match.trainIdx];

            const auto& pt_l  = cv::Vec3d { keypoint_l.pt.x, keypoint_l.pt.y, 1.0f };
            const auto& pt_r  = cv::Vec3d { keypoint_r.pt.x, keypoint_r.pt.y, 1.0f };
            const auto& error = pt_r.t() * camera_matrix.inv().t() * E * camera_matrix.inv() * pt_l;

            if (error[0] > threshold || match.distance > 5)
            {
                continue;
            }

            matches_new.emplace_back(keypoint_l.index, keypoint_r.index, match.distance);
        }
    }

    return matches_new;
}

auto zenslam::utils::process
(
    const frame::sensor&       sensor,
    const calibration&         calibration,
    const class options::slam& options,
    const cv::Ptr<cv::CLAHE>&  clahe
) -> frame::processed
{
    frame::processed processed = { sensor };

    // Convert to grayscale
    std::jthread thread_0
    {
        [&]()
        {
            processed.images[0] = convert_color(sensor.images[0], cv::COLOR_BGR2GRAY);

            if (options.clahe_enabled)
            {
                processed.images[0] = apply_clahe(processed.images[0], clahe);
            }

            processed.undistorted[0] = rectify(processed.images[0], calibration.map_x[0], calibration.map_y[0]);
            processed.pyramids[0]    = pyramid(processed.undistorted[0], options);
        }
    };

    std::jthread thread_1
    {
        [&]()
        {
            processed.images[1] = convert_color(sensor.images[1], cv::COLOR_BGR2GRAY);

            if (options.clahe_enabled)
            {
                processed.images[1] = apply_clahe(processed.images[1], clahe);
            }

            processed.undistorted[1] = rectify(processed.images[1], calibration.map_x[1], calibration.map_y[1]);
            processed.pyramids[1]    = pyramid(processed.undistorted[1], options);
        }
    };

    // IMU pre-integration using preint class
    if (!sensor.imu_data.empty())
    {
        // Create preintegrator with current calibration
        preint imu_preint(calibration.imu, preint::method::ugpm);

        // Configure for optimal UGPM performance
        imu_preint.set_overlap_factor(8);     // 8x state period overlap
        imu_preint.set_state_frequency(50.0); // 50 Hz state frequency
        imu_preint.set_correlate(true);       // Enable correlation

        const double start_t = sensor.imu_data.front().timestamp;
        const double end_t   = std::max(sensor.timestamp, sensor.imu_data.back().timestamp);

#if ZENSLAM_HAS_UGPM
        // Integrate IMU measurements
        auto result = imu_preint.integrate_with_overlap(sensor.imu_data, start_t, end_t);

        if (result.has_value())
        {
            processed.preint = result.value();
        }
        else
        {
            // Fallback to identity if integration fails
            processed.preint = preint::identity();
        }
#else
        // When ugpm is not available, use identity
        processed.preint = { };
#endif
    }
    else
    {
#if ZENSLAM_HAS_UGPM
        // No IMU data - use identity transformation
        processed.preint = preint::identity();
#else
        processed.preint = { };
#endif
    }

    return processed;
}

auto zenslam::utils::solve_pnp
(
    const cv::Matx33d&              camera_matrix,
    const std::vector<cv::Point3d>& points3d,
    const std::vector<cv::Point2d>& points2d,
    cv::Affine3d&                   pose
) -> void
{
    cv::Mat          rvec { pose.rvec() };
    cv::Mat          tvec { pose.translation() };
    std::vector<int> inliers { };

    SPDLOG_INFO("SolvePnP with {} points", points3d.size());

    if (cv::solvePnPRansac(points3d, points2d, camera_matrix, cv::Mat(), rvec, tvec, true, 1000, 4.0, 0.99, inliers))
    {
        SPDLOG_INFO("SolvePnP successful with {} inliers out of {} points", inliers.size(), points3d.size());
        pose = cv::Affine3d(rvec, tvec);
    }
    else
    {
        SPDLOG_WARN("SolvePnP failed");
        throw std::runtime_error("SolvePnP failed");
    }
}

auto zenslam::utils::track
(
    const frame::tracked&                 frame_0,
    const frame::processed&               frame_1,
    const calibration&                    calibration,
    const class options::slam&            options,
    const cv::Ptr<cv::DescriptorMatcher>& matcher
) -> frame::tracked
{
    const auto& detector = grid_detector::create(options);

    map<keypoint> keypoints_0 = { };
    map<keypoint> keypoints_1 = { };
    map<point3d>  points3d    = { };

    {
        std::jthread thread_0
        {
            [&]()
            {
                keypoints_0 += track_keypoints
                (
                    frame_0.pyramids[0],
                    frame_1.pyramids[0],
                    frame_0.keypoints[0],
                    options,
                    calibration.camera_matrix[0]
                );

                if (options.use_parallel_detector)
                {
                    keypoints_0 += detector.detect_keypoints_par(frame_1.undistorted[0], keypoints_0);
                }
                else
                {
                    keypoints_0 += detector.detect_keypoints(frame_1.undistorted[0], keypoints_0);
                }
            }
        };

        std::jthread thread_1
        {
            [&]()
            {
                keypoints_1 += track_keypoints
                (
                    frame_0.pyramids[1],
                    frame_1.pyramids[1],
                    frame_0.keypoints[1],
                    options,
                    calibration.camera_matrix[1]
                );

                if (options.use_parallel_detector)
                {
                    keypoints_1 += detector.detect_keypoints_par(frame_1.undistorted[1], keypoints_1);
                }
                else
                {
                    keypoints_1 += detector.detect_keypoints(frame_1.undistorted[1], keypoints_1);
                }
            }
        };
    }

    keypoints_1 *= match_keypoints
    (
        keypoints_0,
        keypoints_1,
        matcher,
        options
    );

    points3d += triangulate_keypoints
    (
        keypoints_0,
        keypoints_1,
        calibration.projection_matrix[0],
        calibration.projection_matrix[1],
        options.triangulation_reprojection_threshold,
        calibration.cameras[1].pose_in_cam0.translation()
    );

    return { frame_1, keypoints_0, keypoints_1, { }, points3d };
}

auto zenslam::utils::track_keypoints
(
    const std::vector<cv::Mat>&     pyramid_0,
    const std::vector<cv::Mat>&     pyramid_1,
    const map<keypoint>&            keypoints_map_0,
    const class options::slam&      options,
    const cv::Matx33d&              camera_matrix,
    const std::vector<cv::Point2f>& points_1_predicted
) -> std::vector<keypoint>
{
    // Convert previous keypoints to Point
    const auto& keypoints_0 = keypoints_map_0.values() | std::ranges::to<std::vector>();

    if (keypoints_0.empty())
    {
        return { };
    }

    const auto& points_0 = keypoints_map_0.values_sliced<cv::Point2f>
    (
        [](const keypoint& kp)
        {
            return kp.pt;
        }
    ) | std::ranges::to<std::vector>();

    // track points from the previous frame to this frame using the KLT tracker
    // KLT tracking of keypoints from previous frame to current frame (left image)
    auto               points_1 = points_1_predicted;
    std::vector<uchar> status { };
    std::vector<float> errors { };

    cv::calcOpticalFlowPyrLK
    (
        pyramid_0,
        pyramid_1,
        points_0,
        points_1,
        status,
        errors,
        options.klt_window_size,
        options.klt_max_level,
        cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 99, 0.001),
        cv::OPTFLOW_LK_GET_MIN_EIGENVALS
    );

    std::vector<cv::Point2f> points_0_back { };
    std::vector<uchar>       status_back { };
    cv::calcOpticalFlowPyrLK
    (
        pyramid_1,
        pyramid_0,
        points_1,
        points_0_back,
        status_back,
        errors,
        options.klt_window_size,
        options.klt_max_level,
        cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 99, 0.001),
        cv::OPTFLOW_LK_GET_MIN_EIGENVALS
    );

    // Verify KLT tracking results have consistent sizes
    assert(points_0.size() == points_1.size() && points_1.size() == status.size() && status.size() == errors.size());

    // Collect candidates that pass forward-backward check
    std::vector<cv::Point2f> candidate_points_0;
    std::vector<cv::Point2f> candidate_points_1;
    std::vector<size_t>      candidate_indices;

    for (size_t i = 0; i < points_1.size(); ++i)
    {
        if (status[i] && status_back[i] && cv::norm(points_0_back[i] - points_0[i]) < options.klt_threshold)
        {
            candidate_points_0.push_back(points_0[i]);
            candidate_points_1.push_back(points_1[i]);
            candidate_indices.push_back(i);
        }
    }

    // Apply essential matrix filtering if we have enough candidates
    std::vector<keypoint> tracked_keypoints;

    if (candidate_points_0.size() >= 8)
    {
        // Find essential matrix with RANSAC
        std::vector<uchar> inlier_mask;

        auto E = cv::findEssentialMat
        (
            candidate_points_0,
            candidate_points_1,
            camera_matrix,
            cv::RANSAC,
            0.999,
            // Confidence
            options.epipolar_threshold,
            // Threshold in pixels
            inlier_mask
        );

        // Keep only inliers
        for (size_t j = 0; j < candidate_indices.size(); ++j)
        {
            if (inlier_mask[j])
            {
                auto i                = candidate_indices[j];
                auto tracked_keypoint = keypoints_0[i];
                tracked_keypoint.pt   = points_1[i];
                tracked_keypoints.emplace_back(tracked_keypoint);
            }
        }
    }
    else
    {
        // Not enough points for essential matrix, use all candidates
        for (auto i : candidate_indices)
        {
            auto tracked_keypoint = keypoints_0[i];
            tracked_keypoint.pt   = points_1[i];
            tracked_keypoints.emplace_back(tracked_keypoint);
        }
    }

    return tracked_keypoints;
}

auto zenslam::utils::track_keylines
(
    const std::vector<cv::Mat>& pyramid_0,
    const std::vector<cv::Mat>& pyramid_1,
    const map<keyline>&         keylines_map_0,
    const class options::slam&  options
) -> std::vector<keyline>
{
    if (keylines_map_0.empty())
    {
        return { };
    }

    // Extract start and end points of all keylines
    std::vector<cv::Point2f> start_points_0;
    std::vector<cv::Point2f> end_points_0;
    std::vector<keyline>     keylines_0;

    start_points_0.reserve(keylines_map_0.size());
    end_points_0.reserve(keylines_map_0.size());
    keylines_0.reserve(keylines_map_0.size());

    for (const auto& keyline : keylines_map_0 | std::views::values)
    {
        keylines_0.push_back(keyline);
        start_points_0.emplace_back(keyline.startPointX, keyline.startPointY);
        end_points_0.emplace_back(keyline.endPointX, keyline.endPointY);
    }

    // Forward tracking: track start and end points from frame 0 to frame 1
    auto               start_points_1 = start_points_0;
    auto               end_points_1   = end_points_0;
    std::vector<uchar> status_start_fwd;
    std::vector<uchar> status_end_fwd;
    std::vector<float> err_start_fwd;
    std::vector<float> err_end_fwd;

    cv::calcOpticalFlowPyrLK
    (
        pyramid_0,
        pyramid_1,
        start_points_0,
        start_points_1,
        status_start_fwd,
        err_start_fwd,
        options.klt_window_size,
        options.klt_max_level,
        cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 99, 0.001),
        cv::OPTFLOW_LK_GET_MIN_EIGENVALS
    );

    cv::calcOpticalFlowPyrLK
    (
        pyramid_0,
        pyramid_1,
        end_points_0,
        end_points_1,
        status_end_fwd,
        err_end_fwd,
        options.klt_window_size,
        options.klt_max_level,
        cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 99, 0.001),
        cv::OPTFLOW_LK_GET_MIN_EIGENVALS
    );

    // Backward tracking: track from frame 1 back to frame 0
    std::vector<cv::Point2f> start_points_0_back;
    std::vector<cv::Point2f> end_points_0_back;
    std::vector<uchar>       status_start_bwd;
    std::vector<uchar>       status_end_bwd;
    std::vector<float>       err_start_bwd;
    std::vector<float>       err_end_bwd;

    cv::calcOpticalFlowPyrLK
    (
        pyramid_1,
        pyramid_0,
        start_points_1,
        start_points_0_back,
        status_start_bwd,
        err_start_bwd,
        options.klt_window_size,
        options.klt_max_level,
        cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 99, 0.001),
        cv::OPTFLOW_LK_GET_MIN_EIGENVALS
    );

    cv::calcOpticalFlowPyrLK
    (
        pyramid_1,
        pyramid_0,
        end_points_1,
        end_points_0_back,
        status_end_bwd,
        err_end_bwd,
        options.klt_window_size,
        options.klt_max_level,
        cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 99, 0.001),
        cv::OPTFLOW_LK_GET_MIN_EIGENVALS
    );

    // Filter keylines based on forward-backward consistency
    std::vector<keyline> tracked_keylines;
    tracked_keylines.reserve(keylines_0.size());

    for (size_t i = 0; i < keylines_0.size(); ++i)
    {
        // Check if both endpoints were successfully tracked in both directions
        if (status_start_fwd[i] && status_end_fwd[i] &&
            status_start_bwd[i] && status_end_bwd[i])
        {
            // Compute forward-backward error for both endpoints
            const auto fb_error_start = static_cast<float>(cv::norm(start_points_0_back[i] - start_points_0[i]));
            const auto fb_error_end   = static_cast<float>(cv::norm(end_points_0_back[i] - end_points_0[i]));

            // Accept the track only if both endpoints pass the forward-backward threshold
            if (fb_error_start < options.klt_threshold && fb_error_end < options.klt_threshold)
            {
                auto tracked_keyline = keylines_0[i];

                // Update keyline endpoints with tracked positions
                tracked_keyline.startPointX = start_points_1[i].x;
                tracked_keyline.startPointY = start_points_1[i].y;
                tracked_keyline.endPointX   = end_points_1[i].x;
                tracked_keyline.endPointY   = end_points_1[i].y;

                // Update keyline midpoint (pt)
                tracked_keyline.pt.x = (start_points_1[i].x + end_points_1[i].x) * 0.5f;
                tracked_keyline.pt.y = (start_points_1[i].y + end_points_1[i].y) * 0.5f;

                // Update keyline length
                const auto dx              = tracked_keyline.endPointX - tracked_keyline.startPointX;
                const auto dy              = tracked_keyline.endPointY - tracked_keyline.startPointY;
                tracked_keyline.lineLength = std::sqrt(dx * dx + dy * dy);

                // Update keyline angle
                tracked_keyline.angle = std::atan2(dy, dx) * 180.0f / static_cast<float>(CV_PI);

                tracked_keylines.push_back(tracked_keyline);
            }
        }
    }

    return tracked_keylines;
}

auto epipolar_angles(const cv::Vec3d& translation_of_camera1_in_camera0)
{
    return std::views::transform
    (
        [&translation_of_camera1_in_camera0](const cv::Point3d& p)
        {
            const auto vec_to_point_0 = cv::Vec3d(p.x, p.y, p.z);
            const auto vec_to_point_1 = cv::Vec3d(p.x, p.y, p.z) - translation_of_camera1_in_camera0;
            const auto norm_prod      = cv::norm(vec_to_point_0) * cv::norm(vec_to_point_1);
            return norm_prod < 1e-12
                       ? 0.0
                       : std::abs(std::acos(std::clamp(vec_to_point_0.dot(vec_to_point_1) / norm_prod, -1.0, 1.0))) *
                       180 / CV_PI;
        }
    );
}

auto zenslam::utils::triangulate_keypoints
(
    const map<keypoint>& keypoints_0,
    const map<keypoint>& keypoints_1,
    const cv::Matx34d&   projection_0,
    const cv::Matx34d&   projection_1,
    const double         triangulation_threshold,
    const cv::Vec3d&     translation_of_camera1_in_camera0
) -> std::vector<point3d>
{
    const auto& points2f_0 = to_points(keypoints_0.values_matched(keypoints_1) | std::ranges::to<std::vector>());
    const auto& points2f_1 = to_points(keypoints_1.values_matched(keypoints_0) | std::ranges::to<std::vector>());

    const auto& points3d_cv = triangulate_points(points2f_0, points2f_1, projection_0, projection_1);
    const auto& indices     = keypoints_0.keys_matched(keypoints_1) | std::ranges::to<std::vector>();
    const auto& descriptors = keypoints_0.values_matched(keypoints_1) | std::views::transform
    (
        [](const keypoint& kp)
        {
            return kp.descriptor;
        }
    ) | std::ranges::to<std::vector>();

    const auto& points3d_all = point3d::create(points3d_cv, indices, descriptors);

    // Reproject points to compute reprojection error
    const auto& points2f_0_back = project(points3d_cv, projection_0);
    const auto& points2f_1_back = project(points3d_cv, projection_1);

    const auto& errors_0 = vecnorm(points2f_0_back - points2f_0);
    const auto& errors_1 = vecnorm(points2f_1_back - points2f_1);

    const auto& angles = points3d_all | epipolar_angles(translation_of_camera1_in_camera0) | std::ranges::to<
        std::vector>();

    std::vector<point3d> points3d { };
    for (auto i = 0; i < points3d_all.size(); ++i)
    {
        if (points3d_all[i].z > 1 && errors_0[i] < triangulation_threshold && errors_1[i] < triangulation_threshold &&
            angles[i] > 0.25 && angles[i] < 180 -
            0.25) // 4 pixel reprojection error threshold
        {
            points3d.emplace_back(points3d_all[i]);
        }
    }

    return points3d;
}


auto points_0()
{
    return std::views::transform
    (
        [](const zenslam::keyline& kl)
        {
            return kl.getStartPoint();
        }
    );
}

auto points_1()
{
    return std::views::transform
    (
        [](const zenslam::keyline& kl)
        {
            return kl.getEndPoint();
        }
    );
}

auto zenslam::utils::triangulate_keylines
(
    const map<keyline>&        keylines_0,
    const map<keyline>&        keylines_1,
    const cv::Matx34d&         projection_0,
    const cv::Matx34d&         projection_1,
    const class options::slam& options,
    const cv::Vec3d&           translation_of_camera1_in_camera0
) -> std::vector<line3d>
{
    const auto& points2f_0_0 = keylines_0.values_matched(keylines_1) | points_0() | std::ranges::to<std::vector>();
    const auto& points2f_0_1 = keylines_0.values_matched(keylines_1) | points_1() | std::ranges::to<std::vector>();
    const auto& points2f_1_0 = keylines_1.values_matched(keylines_0) | points_0() | std::ranges::to<std::vector>();
    const auto& points2f_1_1 = keylines_1.values_matched(keylines_0) | points_1() | std::ranges::to<std::vector>();

    const auto& points3d_cv_0 = triangulate_points(points2f_0_0, points2f_1_0, projection_0, projection_1);
    const auto& points3d_cv_1 = triangulate_points(points2f_0_1, points2f_1_1, projection_0, projection_1);

    const auto& points2f_0_0_back = project(points3d_cv_0, projection_0);
    const auto& points2f_1_0_back = project(points3d_cv_0, projection_1);
    const auto& points2f_0_1_back = project(points3d_cv_1, projection_0);
    const auto& points2f_1_1_back = project(points3d_cv_1, projection_1);

    const auto& errors_0_0 = vecnorm(points2f_0_0_back - points2f_0_0);
    const auto& errors_1_0 = vecnorm(points2f_1_0_back - points2f_1_0);
    const auto& errors_0_1 = vecnorm(points2f_0_1_back - points2f_0_1);
    const auto& errors_1_1 = vecnorm(points2f_1_1_back - points2f_1_1);

    const auto& angles_0 = points3d_cv_0 | epipolar_angles(translation_of_camera1_in_camera0) | std::ranges::to<
        std::vector>();
    const auto& angles_1 = points3d_cv_1 | epipolar_angles(translation_of_camera1_in_camera0) | std::ranges::to<
        std::vector>();

    const auto& indices_0 = keylines_0.keys_matched(keylines_1) | std::ranges::to<std::vector>();

    std::vector<double> ang;
    for (auto i = 0; i < points3d_cv_0.size(); ++i)
    {
        auto       vector_0  = (points3d_cv_0[i] + points3d_cv_1[i]) / 2.0;
        auto       vector_1  = points3d_cv_0[i] - points3d_cv_1[i];
        const auto norm_prod = cv::norm(vector_0) * cv::norm(vector_1);
        auto       angle     = norm_prod < 1e-12
                                   ? 0.0
                                   : std::abs(std::acos(std::clamp(vector_0.dot(vector_1) / norm_prod, -1.0, 1.0))) *
                                   180 / CV_PI;
        ang.push_back(angle);
    }

    std::vector<line3d> lines3d { };
    for (auto i = 0; i < points3d_cv_0.size(); ++i)
    {
        if (points3d_cv_0[i].z > 1 && points3d_cv_1[i].z > 1 &&
            points3d_cv_0[i].z < 30 && points3d_cv_1[i].z < 30 &&
            errors_0_0[i] < options.triangulation_reprojection_threshold &&
            errors_1_0[i] < options.triangulation_reprojection_threshold &&
            errors_0_1[i] < options.triangulation_reprojection_threshold &&
            errors_1_1[i] < options.triangulation_reprojection_threshold &&
            angles_0[i] > 0.25 && angles_0[i] < 180 - 0.25 &&
            angles_1[i] > 0.25 && angles_1[i] < 180 - 0.25 &&
            ang[i] > 45.0 && ang[i] < 135.0)
        {
            lines3d.emplace_back(std::array { points3d_cv_0[i], points3d_cv_1[i] }, indices_0[i]);
        }
    }

    return lines3d;
}

auto zenslam::utils::triangulate_points
(
    const std::vector<cv::Point2f>& points2f_0,
    const std::vector<cv::Point2f>& points2f_1,
    const cv::Matx34d&              projection_0,
    const cv::Matx34d&              projection_1
) -> std::vector<cv::Point3d>
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

void zenslam::utils::umeyama
(
    const std::vector<cv::Point3d>& src,
    const std::vector<cv::Point3d>& dst,
    cv::Matx33d&                    R,
    cv::Point3d&                    t
)
{
    assert(src.size() == dst.size());
    assert(src.size() >= 3);

    cv::Vec3d mean_src(0, 0, 0), mean_dst(0, 0, 0);
    for (size_t i = 0; i < src.size(); ++i)
    {
        mean_src += cv::Vec3d(src[i].x, src[i].y, src[i].z);
        mean_dst += cv::Vec3d(dst[i].x, dst[i].y, dst[i].z);
    }
    mean_src *= 1.0 / static_cast<double>(src.size());
    mean_dst *= 1.0 / static_cast<double>(dst.size());

    cv::Matx33d Sigma(0, 0, 0, 0, 0, 0, 0, 0, 0);
    for (size_t i = 0; i < src.size(); ++i)
    {
        // Covariance should be E[(dst - mean_dst) * (src - mean_src)^T]
        // i.e. outer product b * a^T so that SVD(C) = U S V^T and R = U * V^T
        auto a = cv::Vec3d(src[i].x, src[i].y, src[i].z) - mean_src;
        auto b = cv::Vec3d(dst[i].x, dst[i].y, dst[i].z) - mean_dst;
        Sigma(0, 0) += b[0] * a[0];
        Sigma(0, 1) += b[0] * a[1];
        Sigma(0, 2) += b[0] * a[2];
        Sigma(1, 0) += b[1] * a[0];
        Sigma(1, 1) += b[1] * a[1];
        Sigma(1, 2) += b[1] * a[2];
        Sigma(2, 0) += b[2] * a[0];
        Sigma(2, 1) += b[2] * a[1];
        Sigma(2, 2) += b[2] * a[2];
    }
    Sigma *= 1.0 / static_cast<double>(src.size());

    cv::Mat Sigma_mat(3, 3, CV_64F);
    for (auto r = 0; r < 3; ++r)
        for (auto c                    = 0; c < 3; ++c)
            Sigma_mat.at<double>(r, c) = Sigma(r, c);

    cv::Mat U_mat, S_mat, Vt_mat;
    cv::SVD::compute(Sigma_mat, S_mat, U_mat, Vt_mat);

    cv::Mat R_mat = U_mat * Vt_mat;
    if (cv::determinant(R_mat) < 0)
    {
        cv::Mat S          = cv::Mat::eye(3, 3, CV_64F);
        S.at<double>(2, 2) = -1.0;
        R_mat              = U_mat * S * Vt_mat;
    }

    R                 = cv::Matx33d(R_mat);
    auto mean_src_rot = cv::Vec3d(R * mean_src);
    t                 = mean_dst - mean_src_rot;
}
