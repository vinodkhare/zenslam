#include "utils_slam.h"

#include <numeric>

#include <gsl/narrow>

#include <opencv2/calib3d.hpp>
#include <opencv2/video/tracking.hpp>

#include <spdlog/spdlog.h>

#include "slam_thread.h"
#include "utils.h"

void zenslam::utils::correspondences
(
    const std::map<size_t, point> &   points,
    const std::map<size_t, keypoint> &keypoints,
    std::vector<cv::Point3d> &        points3d,
    std::vector<cv::Point2d> &        points2d
)
{
    for (const auto &index: keypoints | std::views::keys)
    {
        if (points.contains(index))
        {
            points3d.emplace_back(points.at(index));
            points2d.emplace_back(keypoints.at(index).pt);
        }
    }
}

void zenslam::utils::correspondences_3d3d
(
    const std::map<size_t, point> &points_map_0,
    const std::map<size_t, point> &points_map_1,
    std::vector<cv::Point3d> &     points3d_0,
    std::vector<cv::Point3d> &     points3d_1,
    std::vector<size_t> &          indexes
)
{
    for (const auto &index: points_map_1 | std::views::keys)
    {
        if (points_map_0.contains(index))
        {
            points3d_0.emplace_back(points_map_0.at(index));
            points3d_1.emplace_back(points_map_1.at(index));
            indexes.emplace_back(index);
        }
    }
}

bool zenslam::utils::estimate_rigid
(
    const std::vector<cv::Point3d> &src,
    const std::vector<cv::Point3d> &dst,
    cv::Matx33d &                   R,
    cv::Point3d &                   t
)
{
    if (src.size() != dst.size() || src.size() < 3) return false;

    // Compute means
    auto mean_src = std::accumulate
    (
        src.begin(),
        src.end(),
        cv::Vec3d(0, 0, 0),
        [](const cv::Vec3d &acc, const cv::Point3d &p)
        {
            return acc + cv::Vec3d(p.x, p.y, p.z);
        }
    );
    mean_src *= 1.0 / gsl::narrow<double>(src.size());
    auto mean_dst = std::accumulate
    (
        dst.begin(),
        dst.end(),
        cv::Vec3d(0, 0, 0),
        [](const cv::Vec3d &acc, const cv::Point3d &p)
        {
            return acc + cv::Vec3d(p.x, p.y, p.z);
        }
    );
    mean_dst *= 1.0 / gsl::narrow<double>(dst.size());

    // Compute cross-covariance
    cv::Matx33d Sigma(0, 0, 0, 0, 0, 0, 0, 0, 0);
    for (size_t i = 0; i < src.size(); ++i)
    {
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
    Sigma *= 1.0 / gsl::narrow<double>(src.size());

    // SVD
    cv::Mat Sigma_mat(3, 3,CV_64F);
    for (auto r = 0; r < 3; ++r) for (auto c = 0; c < 3; ++c) Sigma_mat.at<double>(r, c) = Sigma(r, c);
    cv::Mat U, S, Vt;
    cv::SVD::compute(Sigma_mat, S, U, Vt);
    cv::Mat R_mat = U * Vt;
    if (cv::determinant(R_mat) < 0)
    {
        cv::Mat Sfix          = cv::Mat::eye(3, 3,CV_64F);
        Sfix.at<double>(2, 2) = -1.0;
        R_mat                 = U * Sfix * Vt;
    }
    R = cv::Matx33d(R_mat);
    t = mean_dst - cv::Vec3d(R * mean_src);
    return true;
}

auto zenslam::utils::estimate_rigid_ransac
(
    const std::vector<cv::Point3d> &src,
    const std::vector<cv::Point3d> &dst,
    cv::Matx33d &                   best_R,
    cv::Point3d &                   best_t,
    std::vector<size_t> &           inlier_indices,
    std::vector<size_t> &           outlier_indices,
    std::vector<double> &           errors,
    const double                    threshold,
    const int                       max_iterations,
    const int                       min_inliers
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

    for (int iter = 0; iter < max_iterations; ++iter)
    {
        // Randomly sample 3 unique indices
        std::set<size_t> idx_set;
        while (idx_set.size() < 3) idx_set.insert(dist(rng));
        std::vector<size_t> indices(idx_set.begin(), idx_set.end());

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
            errs.push_back(err);
            if (err < threshold) inliers.push_back(i);
        }

        if (inliers.size() > best_inlier_count && inliers.size() >= static_cast<size_t>(min_inliers))
        {
            best_inlier_count = static_cast<int>(inliers.size());
            bestR             = R;
            bestt             = t;
            best_inliers      = inliers;
            best_errors       = errs;
        }
    }

    if (best_inlier_count >= min_inliers)
    {
        best_R         = bestR;
        best_t         = bestt;
        inlier_indices = best_inliers;
        outlier_indices.clear();
        for (size_t i = 0; i < src.size(); ++i)
        {
            if (std::ranges::find(inlier_indices, i) == inlier_indices.end()) outlier_indices.push_back(i);
        }
        errors = best_errors;
        return true;
    }
    return false;
}

auto zenslam::utils::filter
(
    const std::vector<cv::KeyPoint> &keypoints0,
    const std::vector<cv::KeyPoint> &keypoints1,
    const std::vector<cv::DMatch> &  matches,
    const cv::Matx33d &              fundamental,
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
            const auto &pt0   = points0[i];
            const auto &pt1   = points1[i];
            const auto &line1 = epilines1[i];
            const auto &line0 = epilines0[i];

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

auto zenslam::utils::match
(
    const std::map<size_t, keypoint> &keypoints_0,
    std::map<size_t, keypoint> &      keypoints_1,
    const cv::Matx33d &               fundamental,
    double                            epipolar_threshold
) -> void
{
    cv::Mat                 descriptors_l { };
    cv::Mat                 descriptors_r { };
    std::vector<keypoint>   unmatched_l { };
    std::vector<keypoint>   unmatched_r { };
    std::vector<cv::DMatch> matches_existing { };

    for (const auto &keypoint_l: keypoints_0 | std::views::values)
    {
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

    for (const auto &keypoint_r: keypoints_1 | std::views::values)
    {
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

    for (auto query_index = 0; query_index < keypoints_0.size(); ++query_index)
    {
        for (auto train_index = 0; train_index < keypoints_1.size(); ++train_index)
        {
            if (unmatched_l[query_index].index == unmatched_r[train_index].index)
            {
                matches_existing.emplace_back(query_index, train_index, 0);
            }
        }
    }

    const cv::BFMatcher     matcher { cv::NORM_L2, true };
    std::vector<cv::DMatch> matches;
    matcher.match(descriptors_l, descriptors_r, matches);

    auto matches_map = matches | std::views::transform
                       (
                           [](const auto &match)
                           {
                               return std::make_pair(match.queryIdx, match.trainIdx);
                           }
                       ) | std::ranges::to<std::map<int, int>>();

    auto remove_count = 0;
    for (const auto &match: matches_existing)
    {
        if (!matches_map.contains(match.queryIdx))
        {
            keypoints_1.erase(unmatched_r[match.trainIdx].index);
            remove_count++;
        }
    }

    SPDLOG_INFO("Removed {} matches", remove_count);

    SPDLOG_INFO("matches count: {}", matches.size());

    matches =
            filter
            (
                utils::cast<cv::KeyPoint>(unmatched_l),
                utils::cast<cv::KeyPoint>(unmatched_r),
                matches,
                fundamental,
                epipolar_threshold
            );

    SPDLOG_INFO("matches filtered count: {}", matches.size());

    for (const auto &match: matches)
    {
        const auto &keypoint_l = unmatched_l[match.queryIdx];
        auto        keypoint_r = unmatched_r[match.trainIdx];

        keypoints_1.erase(keypoint_r.index);

        keypoint_r.index = keypoint_l.index;

        keypoints_1[keypoint_r.index] = keypoint_r;
    }
}

auto zenslam::utils::solve_pnp
(
    const cv::Matx33d &             camera_matrix,
    const std::vector<cv::Point3d> &points3d,
    const std::vector<cv::Point2d> &points2d,
    cv::Affine3d &                  pose
) -> void
{
    cv::Mat          rvec { pose.rvec() };
    cv::Mat          tvec { pose.translation() };
    std::vector<int> inliers { };

    SPDLOG_INFO("SolvePnP with {} points", points3d.size());

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
            4.0,
            0.99,
            inliers
        ))
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

void zenslam::utils::track(const mono_frame &frame_0, mono_frame &frame_1, const class options::slam &options)
{
    // track points from the previous frame to this frame using the KLT tracker
    // KLT tracking of keypoints from previous frame to current frame (left image)
    std::vector<cv::Point2f> points_1 { };
    std::vector<uchar>       status { };
    std::vector<float>       err { };

    // Convert previous keypoints to Point
    const auto &keypoints_0 = values(frame_0.keypoints);
    const auto &points_0    = to_points(frame_0.keypoints);

    if (keypoints_0.empty())
    {
        return;
    }

    cv::calcOpticalFlowPyrLK
    (
        frame_0.undistorted,
        frame_1.undistorted,
        points_0,
        points_1,
        status,
        err,
        options.klt_window_size,
        options.klt_max_level,
        cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 99, 0.01),
        cv::OPTFLOW_LK_GET_MIN_EIGENVALS
    );

    // Verify KLT tracking results have consistent sizes
    assert(points_0.size() == points_1.size() && points_1.size() == status.size() && status.size() == err.size());

    // Update frame.l.keypoints with tracked points
    for (size_t i = 0; i < points_1.size(); ++i)
    {
        if (status[i] && std::abs(points_1[i].y - points_0[i].y) < 32)
        {
            frame_1.keypoints[keypoints_0[i].index]    = keypoints_0[i];
            frame_1.keypoints[keypoints_0[i].index].pt = points_1[i];
        }
    }
}


auto zenslam::utils::triangulate
(
    stereo_frame &                  frame,
    const cv::Matx34d &             projection_l,
    const cv::Matx34d &             projection_r,
    std::map<unsigned long, point> &points
) -> void
{
    std::vector<cv::Point2f> points_l { };
    std::vector<cv::Point2f> points_r { };
    std::vector<size_t>      indices { };

    for (const auto &[index, keypoint_L]: frame.l.keypoints)
    {
        if (frame.r.keypoints.contains(index))
        {
            const auto &keypoint_R = frame.r.keypoints.at(index);

            points_l.emplace_back(keypoint_L.pt);
            points_r.emplace_back(keypoint_R.pt);

            indices.emplace_back(index);
        }
    }

    cv::Mat points4d;
    cv::triangulatePoints(projection_l, projection_r, points_l, points_r, points4d);

    std::vector<cv::Point3d> points3d;
    points3d.reserve(points4d.cols);

    for (auto c = 0; c < points4d.cols; ++c)
    {
        cv::Vec4d point4d = points4d.col(c);

        if (std::abs(point4d[3]) > 1e-9)
        {
            points.emplace
            (
                indices[c],
                point { { point4d[0] / point4d[3], point4d[1] / point4d[3], point4d[2] / point4d[3] }, indices[c] }
            );
        }
    }

    // What we need to do at this point is to reproject the 3D points back onto both images
    // and check the reprojection error. If the error is too high, we should discard the point.
    // This will help to filter out points that are not well triangulated due to noise or other issues.

    // Reproject and prune points with large reprojection error
    constexpr auto             reprojection_threshold = 1.0; // pixels
    std::vector<unsigned long> to_erase;
    for (auto c = 0; c < points4d.cols; ++c)
    {
        cv::Vec4d X = points4d.col(c);
        if (std::abs(X[3]) <= 1e-9)
        {
            to_erase.push_back(indices[c]);
            continue;
        }

        auto proj_l_h = projection_l * X;
        auto proj_r_h = projection_r * X;

        if (std::abs(proj_l_h[2]) <= 1e-9 || std::abs
            (proj_r_h[2]) <= 1e-9 || points[indices[c]].z < 1 || points[indices[c]].z > 100)
        {
            to_erase.push_back(indices[c]);
            continue;
        }

        const cv::Point2d reproj_l { proj_l_h[0] / proj_l_h[2], proj_l_h[1] / proj_l_h[2] };
        const cv::Point2d reproj_r { proj_r_h[0] / proj_r_h[2], proj_r_h[1] / proj_r_h[2] };

        const auto &orig_l = points_l[c];
        const auto &orig_r = points_r[c];

        const auto err_l = std::hypot(reproj_l.x - orig_l.x, reproj_l.y - orig_l.y);
        const auto err_r = std::hypot(reproj_r.x - orig_r.x, reproj_r.y - orig_r.y);
        const auto err   = 0.5 * (err_l + err_r);

        if (!std::isfinite(err) || err > reprojection_threshold)
        {
            SPDLOG_INFO("Reprojection error for point {}: {:.3f} pixels", indices[c], err);

            to_erase.push_back(indices[c]);
        }
    }

    for (auto id: to_erase)
    {
        points.erase(id);
        frame.l.keypoints.erase(id);
        frame.r.keypoints.erase(id);
    }
}

void zenslam::utils::umeyama
(
    const std::vector<cv::Point3d> &src,
    const std::vector<cv::Point3d> &dst,
    cv::Matx33d &                   R,
    cv::Point3d &                   t
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

auto zenslam::utils::undistort(const cv::Mat &image, const calibration &calibration) -> cv::Mat
{
    cv::Mat undistorted { };

    switch (calibration.distortion_model)
    {
        case calibration::distortion_model::radial_tangential:
            cv::undistort
            (
                image,
                calibration.camera_matrix(),
                calibration.distortion_coefficients,
                undistorted,
                calibration.camera_matrix()
            );
            break;

        case calibration::distortion_model::equidistant:
            cv::fisheye::undistortImage
            (
                image,
                undistorted,
                calibration.camera_matrix(),
                calibration.distortion_coefficients,
                calibration.camera_matrix()
            );
            break;
    }

    return undistorted;
}
