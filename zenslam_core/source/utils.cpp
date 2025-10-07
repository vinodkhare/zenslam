#include "utils.h"

#include <chrono>
#include <ranges>

#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>

#include <spdlog/spdlog.h>

auto zenslam::utils::draw_keypoints(const mono_frame &frame) -> cv::Mat
{
    const auto &keypoints = utils::cast<cv::KeyPoint>(values(frame.keypoints_));

    // DRAW_RICH_KEYPOINTS shows size and orientation
    cv::Mat keypoints_image { };

    cv::drawKeypoints
    (
        frame.undistorted,
        keypoints,
        keypoints_image,
        cv::Scalar(0, 255, 0),
        cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS
    );

    return keypoints_image;
}

auto zenslam::utils::draw_matches(const stereo_frame &frame) -> cv::Mat
{
    cv::Mat matches_image { };

    const auto &keypoints_l = values(frame.l.keypoints_);
    const auto &keypoints_r = values(frame.r.keypoints_);

    std::vector<cv::DMatch> matches { };

    for (auto query = 0; query < keypoints_l.size(); query++)
    {
        for (auto train = 0; train < keypoints_r.size(); train++)
        {
            if (keypoints_l[query].index == keypoints_r[train].index)
            {
                matches.emplace_back(query, train, 1.0);
            }
        }
    }

    cv::drawMatches
    (
        frame.l.undistorted,
        utils::cast<cv::KeyPoint>(keypoints_l),
        frame.r.undistorted,
        utils::cast<cv::KeyPoint>(keypoints_r),
        matches,
        matches_image,
        cv::Scalar(000, 255, 000),
        cv::Scalar(255, 000, 000),
        std::vector<char>(),
        cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS
    );

    return matches_image;
}

auto zenslam::utils::draw_matches(const mono_frame &frame_0, const mono_frame &frame_1) -> cv::Mat
{
    cv::Mat matches_image { };

    const auto &keypoints_0 = values(frame_0.keypoints_);
    const auto &keypoints_1 = values(frame_1.keypoints_);

    std::vector<cv::DMatch> matches { };

    for (auto query = 0; query < keypoints_0.size(); query++)
    {
        for (auto train = 0; train < keypoints_1.size(); train++)
        {
            if (keypoints_0[query].index == keypoints_1[train].index)
            {
                matches.emplace_back(query, train, 0);
            }
        }
    }

    cv::drawMatches
    (
        frame_0.undistorted,
        utils::cast<cv::KeyPoint>(keypoints_0),
        frame_1.undistorted,
        utils::cast<cv::KeyPoint>(keypoints_1),
        matches,
        matches_image,
        cv::Scalar(000, 255, 000),
        cv::Scalar(255, 000, 000),
        std::vector<char>(),
        cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS
    );

    return matches_image;
}

auto zenslam::utils::skew(const cv::Vec3d &vector) -> cv::Matx33d
{
    auto skew = cv::Matx33d::zeros();

    skew(0, 1) = -vector[2];
    skew(1, 0) = vector[2];
    skew(0, 2) = vector[1];
    skew(2, 0) = -vector[1];
    skew(1, 2) = -vector[0];
    skew(2, 1) = vector[0];

    return skew;
}

auto zenslam::utils::to_keypoints(const std::vector<keypoint> &keypoints) -> std::vector<cv::KeyPoint>
{
    std::vector<cv::KeyPoint> cv_keypoints;
    cv_keypoints.reserve(keypoints.size());

    std::ranges::transform
    (
        keypoints,
        std::back_inserter(cv_keypoints),
        [](const auto &keypoint)
        {
            return keypoint;
        }
    );

    return cv_keypoints;
}

auto zenslam::utils::to_map(const std::vector<cv::DMatch> &matches) -> std::map<int, int>
{
    std::map<int, int> map;
    std::ranges::transform
    (
        matches,
        std::inserter(map, map.begin()),
        [](const auto &match)
        {
            return std::make_pair(match.queryIdx, match.trainIdx);
        }
    );
    return map;
}

auto zenslam::utils::to_points
(
    const std::vector<cv::KeyPoint> &keypoints0,
    const std::vector<cv::KeyPoint> &keypoints1,
    const std::vector<cv::DMatch> &  matches
) -> auto
{
    std::vector<cv::Point2f> points0;
    points0.reserve(matches.size());

    std::vector<cv::Point2f> points1;
    points1.reserve(matches.size());

    for (const auto &match: matches)
    {
        points0.push_back(keypoints0[match.queryIdx].pt);
        points1.push_back(keypoints1[match.trainIdx].pt);
    }

    return std::make_tuple(points0, points1);
}

auto zenslam::utils::to_points(const std::vector<cv::KeyPoint> &keypoints) -> std::vector<cv::Point2f>
{
    std::vector<cv::Point2f> points;
    cv::KeyPoint::convert(keypoints, points);
    return points;
}

auto zenslam::utils::to_points(const std::map<size_t, keypoint> &keypoints) -> std::vector<cv::Point2f>
{
    std::vector<cv::Point2f> points;
    points.reserve(keypoints.size());

    for (const auto &value: keypoints | std::views::values)
    {
        points.emplace_back(value.pt);
    }

    return points;
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

    for (const auto &[index, keypoint_L]: frame.l.keypoints_)
    {
        if (frame.r.keypoints_.contains(index))
        {
            const auto &keypoint_R = frame.r.keypoints_.at(index);

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

        if (std::abs(proj_l_h[2]) <= 1e-9 || std::abs(proj_r_h[2]) <= 1e-9 || points[indices[c]].z < 1 || points[indices[c]].z > 100)
        {
            to_erase.push_back(indices[c]);
            continue;
        }

        cv::Point2d reproj_l { proj_l_h[0] / proj_l_h[2], proj_l_h[1] / proj_l_h[2] };
        cv::Point2d reproj_r { proj_r_h[0] / proj_r_h[2], proj_r_h[1] / proj_r_h[2] };

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
        frame.l.keypoints_.erase(id);
        frame.r.keypoints_.erase(id);
    }
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
    mean_src *= (1.0 / src.size());
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
    mean_dst *= (1.0 / dst.size());

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
    Sigma *= (1.0 / src.size());

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
