#include "zenslam/utils_slam.h"

#include <map>
#include <numeric>
#include <random>
#include <tuple>
#include <vector>

#include <gsl/narrow>

#include <opencv2/calib3d.hpp>
#include <opencv2/video/tracking.hpp>

#include <spdlog/spdlog.h>

#include "zenslam/pose_data.h"
#include "zenslam/utils.h"
#include "zenslam/utils_opencv.h"
#include "zenslam/frame/stereo.h"

void zenslam::utils::correspondences_3d2d
(
    const std::map<size_t, point3d>&  points,
    const std::map<size_t, keypoint>& keypoints,
    std::vector<cv::Point3d>&         points3d,
    std::vector<cv::Point2d>&         points2d,
    std::vector<size_t>&              indices
)
{
    for (const auto& index: keypoints | std::views::keys)
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
    for (const auto& index: points_map_1 | std::views::keys)
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

            for (auto i: inliers) pose_data.inliers.push_back(indices[i]);

            auto inliers_set = std::set(pose_data.inliers.begin(), pose_data.inliers.end());
            for (auto i: indices) if (!inliers_set.contains(i)) pose_data.outliers.push_back(i);

            auto points2d_back = std::vector<cv::Point2d> { };
            cv::projectPoints(points3d, rvec, tvec, camera_matrix, cv::Mat(), points2d_back);

            for (auto i: inliers)
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
    if (points3d_0.size() != points3d_1.size() || points3d_0.size() < 3) return false;

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
    for (auto r = 0; r < 3; ++r) for (auto c = 0; c < 3; ++c) Sigma_mat.at<double>(r, c) = Sigma(r, c);
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
    if (src.size() != dst.size() || src.size() < 3) return false;

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
        while (idx_set.size() < 3) idx_set.insert(dist(rng));
        std::vector indices(idx_set.begin(), idx_set.end());

        // Build minimal sets
        std::vector<cv::Point3d> src_sample, dst_sample;
        for (const auto i: indices)
        {
            src_sample.push_back(src[i]);
            dst_sample.push_back(dst[i]);
        }

        cv::Matx33d R;
        cv::Point3d t;
        if (!estimate_rigid(src_sample, dst_sample, R, t)) continue;

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
            if (std::ranges::find(inlier_indices, i) == inlier_indices.end()) outlier_indices.push_back(i);
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

auto zenslam::utils::match_keypoints
(
    const map<keypoint>& map_keypoints_l,
    const map<keypoint>& map_keypoints_r,
    const cv::Matx33d&   fundamental,
    double               epipolar_threshold
) -> std::vector<cv::DMatch>
{
    cv::Mat                 descriptors_l { };
    cv::Mat                 descriptors_r { };
    std::vector<keypoint>   unmatched_l { };
    std::vector<keypoint>   unmatched_r { };
    std::vector<cv::DMatch> matches_new { };

    for (const auto& keypoint_l: map_keypoints_l | std::views::values)
    {
        if (map_keypoints_r.contains(keypoint_l.index)) continue;

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

    for (const auto& keypoint_r: map_keypoints_r | std::views::values)
    {
        if (map_keypoints_l.contains(keypoint_r.index)) continue;

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

    const cv::BFMatcher     matcher { descriptors_l.depth() == CV_8U ? cv::NORM_HAMMING : cv::NORM_L2, true };
    std::vector<cv::DMatch> matches;
    matcher.match(descriptors_l, descriptors_r, matches);

    matches =
            filter
            (
                utils::cast<cv::KeyPoint>(unmatched_l),
                utils::cast<cv::KeyPoint>(unmatched_r),
                matches,
                fundamental,
                epipolar_threshold
            );

    for (const auto& match: matches)
    {
        const auto& keypoint_l = unmatched_l[match.queryIdx];
        const auto& keypoint_r = unmatched_r[match.trainIdx];

        matches_new.emplace_back(keypoint_l.index, keypoint_r.index, match.distance);
    }

    return matches_new;
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

    for (const auto& kl: keylines_map_0 | std::views::values)
    {
        keylines_0.push_back(kl);
        if (descriptors_0.empty()) descriptors_0 = kl.descriptor;
        else cv::vconcat(descriptors_0, kl.descriptor, descriptors_0);
    }

    for (const auto& kl: keylines_map_1 | std::views::values)
    {
        keylines_1.push_back(kl);
        if (descriptors_1.empty()) descriptors_1 = kl.descriptor;
        else cv::vconcat(descriptors_1, kl.descriptor, descriptors_1);
    }

    // Match descriptors
    const cv::BFMatcher     matcher(cv::NORM_HAMMING, true);
    std::vector<cv::DMatch> matches;
    if (!descriptors_0.empty() && !descriptors_1.empty()) matcher.match(descriptors_0, descriptors_1, matches);

    // Epipolar filtering on endpoints and midpoint
    std::vector<cv::DMatch> filtered;
    filtered.reserve(matches.size());

    for (const auto& match: matches)
    {
        const auto& kl0 = keylines_0[match.queryIdx];
        const auto& kl1 = keylines_1[match.trainIdx];

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

    for (const auto& keypoint_0: keypoints_map_0 | std::views::values)
    {
        if (keypoints_map_1.contains(keypoint_0.index)) continue;

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

    for (const auto& keypoint_1: keypoints_map_1 | std::views::values)
    {
        if (keypoints_map_0.contains(keypoint_1.index)) continue;

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

    for (const auto& [match, msk]: std::ranges::views::zip(matches, mask))
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

auto zenslam::utils::pre_process
(
    const frame::camera&       frame,
    const camera_calibration&  calibration,
    const class options::slam& options,
    const cv::Ptr<cv::CLAHE>&  clahe
) -> frame::camera
{
    auto result = frame;

    result.image = convert_color(result.image, cv::COLOR_BGR2GRAY);

    if (options.clahe_enabled)
    {
        result.image = apply_clahe(result.image, clahe);
    }

    result.undistorted = utils::undistort(result.image, calibration);
    result.pyramid     = pyramid(result.undistorted, options);

    return result;
}

auto zenslam::utils::pre_process
(
    const frame::stereo&                     frame,
    const std::array<camera_calibration, 2>& calibration,
    const class options::slam&               options,
    const cv::Ptr<cv::CLAHE>&                clahe
) -> frame::stereo
{
    auto result = frame;

    result.cameras[0] = pre_process(result.cameras[0], calibration[0], options, clahe);
    result.cameras[1] = pre_process(result.cameras[1], calibration[1], options, clahe);

    return result;
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

auto zenslam::utils::track_keypoints
(
    const std::vector<cv::Mat>&     pyramid_0,
    const std::vector<cv::Mat>&     pyramid_1,
    const map<keypoint>&            keypoints_map_0,
    const class options::slam&      options,
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

    std::vector<keypoint> tracked_keypoints { };
    for (size_t i = 0; i < points_1.size(); ++i)
    {
        if (status[i] && status_back[i] && cv::norm(points_0_back[i] - points_0[i]) < options.klt_threshold)
        {
            auto tracked_keypoint = keypoints_0[i];
            tracked_keypoint.pt   = points_1[i];

            tracked_keypoints.emplace_back(tracked_keypoint);
        }
    }

    return tracked_keypoints;
}

auto zenslam::utils::track
(
    const std::array<frame::stereo, 2>& frames,
    const class options::slam&          options
) -> std::array<std::vector<keypoint>, 2>
{
    return
    {
        track_keypoints
        (
            frames[0].cameras[0].pyramid,
            frames[1].cameras[0].pyramid,
            frames[0].cameras[0].keypoints,
            options
        ),

        track_keypoints
        (
            frames[0].cameras[1].pyramid,
            frames[1].cameras[1].pyramid,
            frames[0].cameras[1].keypoints,
            options
        )
    };
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

    for (const auto& keyline: keylines_map_0 | std::views::values)
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

auto zenslam::utils::triangulate_keypoints
(
    const map<keypoint>& keypoints_0,
    const map<keypoint>& keypoints_1,
    const cv::Matx34d& projection_0,
    const cv::Matx34d& projection_1,
    const double triangulation_threshold
) -> std::vector<point3d>
{
    const auto& points2f_0  = to_points(keypoints_0.values_matched(keypoints_1) | std::ranges::to<std::vector>());
    const auto& points2f_1  = to_points(keypoints_1.values_matched(keypoints_0) | std::ranges::to<std::vector>());
    const auto& points3d_cv = triangulate_points(points2f_0, points2f_1, projection_0, projection_1);

    const auto& indices  = keypoints_0.keys_matched(keypoints_1) | std::ranges::to<std::vector>();
    const auto& points3d_all = point3d::create(points3d_cv, indices);

    // Reproject points to compute reprojection error
    const auto& points2f_0_back = project(points3d_cv, projection_0);
    const auto& points2f_1_back = project(points3d_cv, projection_1);

    const auto& errors_0 = vecnorm(points2f_0_back - points2f_0);
    const auto& errors_1 = vecnorm(points2f_1_back - points2f_1);

    std::vector<point3d> points3d { };
    for (auto i = 0; i < points3d_all.size(); ++i)
    {
        if (points3d_all[i].z > 1 && errors_0[i] < triangulation_threshold && errors_1[i] < triangulation_threshold) // 4 pixel reprojection error threshold
        {
            points3d.emplace_back(points3d_all[i]);
        }
    }

    return points3d;
}


auto zenslam::utils::triangulate_keylines
(
    const map<keyline>&       keylines_l,
    const map<keyline>&       keylines_r,
    const cv::Matx34d&        P_l,
    const cv::Matx34d&        P_r,
    const class options::slam& options
) -> std::vector<line3d>
{
    std::vector<line3d> lines3d { };

    const auto& matched_keylines_l = keylines_l.values_matched(keylines_r) | std::ranges::to<std::vector>();
    const auto& matched_keylines_r = keylines_r.values_matched(keylines_l) | std::ranges::to<std::vector>();

    if (matched_keylines_l.size() != matched_keylines_r.size()) return lines3d;

    constexpr auto reproj_thresh = 1.0; // pixels
    // Minimum triangulation angle (in radians). 1 degree is a common conservative choice.
    constexpr auto tri_min = 1.0 * CV_PI / 180.0;

    // Compute camera centers from projection matrices via SVD (P * C_h = 0)
    auto camera_center = [](const cv::Matx34d& P) -> cv::Point3d
    {
        cv::Mat Pm(3, 4, CV_64F);
        for (auto r = 0; r < 3; ++r) for (auto c = 0; c < 4; ++c) Pm.at<double>(r, c) = P(r, c);
        cv::Mat w, u, vt;
        cv::SVD::compute(Pm, w, u, vt, cv::SVD::FULL_UV);
        cv::Mat v = vt.row(3).t();
        const auto wv = v.at<double>(3);
        return {v.at<double>(0) / wv, v.at<double>(1) / wv, v.at<double>(2) / wv};
    };

    const auto C_l = camera_center(P_l);
    const auto C_r = camera_center(P_r);

    auto triangulation_angle = [&](const cv::Point3d& X) -> double
    {
        const cv::Vec3d a(C_l.x - X.x, C_l.y - X.y, C_l.z - X.z);
        const cv::Vec3d b(C_r.x - X.x, C_r.y - X.y, C_r.z - X.z);
        const auto na = cv::norm(a), nb = cv::norm(b);
        const auto denom = na * nb;
        if (denom < 1e-12) return 0.0;
        auto c = a.dot(b) / denom;
        c = std::clamp(c, -1.0, 1.0);
        return std::acos(c);
    };

    for (size_t i = 0; i < matched_keylines_l.size(); ++i)
    {
        const auto& kl_l = matched_keylines_l[i];
        const auto& kl_r = matched_keylines_r[i];

        const cv::Point2f pl0(kl_l.startPointX, kl_l.startPointY);
        const cv::Point2f pl1(kl_l.endPointX,   kl_l.endPointY);
        const cv::Point2f pr0(kl_r.startPointX, kl_r.startPointY);
        const cv::Point2f pr1(kl_r.endPointX,   kl_r.endPointY);

        // Hypothesis A: (pl0<->pr0, pl1<->pr1)
        const auto X0_A = triangulate_points(std::vector<cv::Point2f>{pl0}, std::vector<cv::Point2f>{pr0}, P_l, P_r);
        const auto X1_A = triangulate_points(std::vector<cv::Point2f>{pl1}, std::vector<cv::Point2f>{pr1}, P_l, P_r);

        // Hypothesis B: (pl0<->pr1, pl1<->pr0)
        const auto X0_B = triangulate_points(std::vector<cv::Point2f>{pl0}, std::vector<cv::Point2f>{pr1}, P_l, P_r);
        const auto X1_B = triangulate_points(std::vector<cv::Point2f>{pl1}, std::vector<cv::Point2f>{pr0}, P_l, P_r);

        auto reproj_error = [&](const cv::Point3d& X0, const cv::Point3d& X1) -> double
        {
            const auto pl0b = project(std::vector<cv::Point3d>{X0}, P_l)[0];
            const auto pl1b = project(std::vector<cv::Point3d>{X1}, P_l)[0];
            const auto pr0b = project(std::vector<cv::Point3d>{X0}, P_r)[0];
            const auto pr1b = project(std::vector<cv::Point3d>{X1}, P_r)[0];

            const auto err_l0 = cv::norm(pl0b - cv::Point2d(pl0));
            const auto err_l1 = cv::norm(pl1b - cv::Point2d(pl1));
            const auto err_r0 = cv::norm(pr0b - cv::Point2d(pr0));
            const auto err_r1 = cv::norm(pr1b - cv::Point2d(pr1));
            return 0.25 * (err_l0 + err_l1 + err_r0 + err_r1);
        };

        auto best_err = std::numeric_limits<double>::infinity();
        cv::Point3d best_X0, best_X1;

        auto depth_l = [&](const cv::Point3d& X) -> double
        {
            const cv::Vec4d Xh(X.x, X.y, X.z, 1.0);
            const auto pl = P_l * Xh;
            return pl[2];
        };

        auto depth_r = [&](const cv::Point3d& X) -> double
        {
            const cv::Vec4d Xh(X.x, X.y, X.z, 1.0);
            const auto pr = P_r * Xh;
            return pr[2];
        };

        auto disparity = [&](const cv::Point2f& a_l, const cv::Point2f& a_r,
                             const cv::Point2f& b_l, const cv::Point2f& b_r) -> double
        {
            // If stereo is rectified, x-disparity is meaningful; otherwise use Euclidean as fallback
            const auto d0 = std::abs(static_cast<double>(a_l.x) - static_cast<double>(a_r.x));
            const auto d1 = std::abs(static_cast<double>(b_l.x) - static_cast<double>(b_r.x));
            return 0.5 * (d0 + d1);
        };

        const auto disp_A = disparity(pl0, pr0, pl1, pr1);
        const auto disp_B = disparity(pl0, pr1, pl1, pr0);

        constexpr auto disp_min = 8.0; // px; tune if needed

        if (!X0_A.empty() && !X1_A.empty() && X0_A[0].z > 0.0 && X1_A[0].z > 0.0 && disp_A >= disp_min)
        {
            const auto eA = reproj_error(X0_A[0], X1_A[0]);
            if (eA < best_err) { best_err = eA; best_X0 = X0_A[0]; best_X1 = X1_A[0]; }
        }
        if (!X0_B.empty() && !X1_B.empty() && X0_B[0].z > 0.0 && X1_B[0].z > 0.0 && disp_B >= disp_min)
        {
            const auto eB = reproj_error(X0_B[0], X1_B[0]);
            if (eB < best_err) { best_err = eB; best_X0 = X0_B[0]; best_X1 = X1_B[0]; }
        }

        // Cheirality for both cameras + triangulation angle gate at both endpoints
        const auto ang0 = (std::isfinite(best_err) ? triangulation_angle(best_X0) : 0.0);
        const auto ang1 = (std::isfinite(best_err) ? triangulation_angle(best_X1) : 0.0);
        if (best_err < reproj_thresh &&
            depth_l(best_X0) > 0.0 && depth_r(best_X0) > 0.0 &&
            depth_l(best_X1) > 0.0 && depth_r(best_X1) > 0.0 &&
            std::min(ang0, ang1) >= tri_min)
        {
            // Optional clipping to [min_depth, max_depth]
            auto clip_depth = [&](const cv::Point3d& X) -> cv::Point3d
            {
                // Clip in camera-L depth; if outside, project along the segment to the nearest depth bound
                const auto z = best_X1.z - best_X0.z;
                auto clampZ = [&](double desiredZ) -> cv::Point3d
                {
                    const auto t = std::abs(z) > 1e-9 ? (desiredZ - best_X0.z) / z : 0.0;
                    const auto tc = std::clamp(t, 0.0, 1.0);
                    return {
                        best_X0.x + tc * (best_X1.x - best_X0.x),
                        best_X0.y + tc * (best_X1.y - best_X0.y),
                        best_X0.z + tc * (best_X1.z - best_X0.z)
                    };
                };

                const auto zmin = options.min_depth;
                const auto zmax = options.max_depth;

                if (X.z < zmin) return clampZ(zmin);
                if (X.z > zmax) return clampZ(zmax);
                return X;
            };

            auto X0c = clip_depth(best_X0);
            auto X1c = clip_depth(best_X1);

            // If clipping inverted the order into a degenerate segment, keep original
            if (cv::norm(cv::Vec3d(X1c.x - X0c.x, X1c.y - X0c.y, X1c.z - X0c.z)) < 1e-6)
            {
                X0c = best_X0; X1c = best_X1;
            }

            lines3d.emplace_back(line3d{X0c, X1c, kl_l.index});
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
        if (std::abs(X[3]) > 1E-9) points3d.emplace_back(X[0] / X[3], X[1] / X[3], X[2] / X[3]);
        else points3d.emplace_back(0, 0, 0);
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
    mean_src *= (1.0 / static_cast<double>(src.size()));
    mean_dst *= (1.0 / static_cast<double>(dst.size()));

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
    Sigma *= (1.0 / static_cast<double>(src.size()));

    cv::Mat Sigma_mat(3, 3, CV_64F);
    for (auto r = 0; r < 3; ++r) for (auto c = 0; c < 3; ++c) Sigma_mat.at<double>(r, c) = Sigma(r, c);

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

auto zenslam::utils::undistort(const cv::Mat& image, const camera_calibration& calibration) -> cv::Mat
{
    cv::Mat undistorted { };
    cv::remap(image, undistorted, calibration.map_x, calibration.map_y, cv::INTER_CUBIC);
    return undistorted;
}
