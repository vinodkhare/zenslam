#include "zenslam/rigid_transform.h"

#include <numeric>
#include <random>
#include <set>
#include <vector>

#include <gsl/narrow>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

bool zenslam::utils::estimate_rigid(
    const std::vector<cv::Point3d>& points3d_0,
    const std::vector<cv::Point3d>& points3d_1,
    cv::Matx33d&                    R,
    cv::Point3d&                    t)
{
    if (points3d_0.size() != points3d_1.size() || points3d_0.size() < 3)
        return false;

    // Compute means
    auto mean_src = std::accumulate(
                        points3d_0.begin(),
                        points3d_0.end(),
                        cv::Vec3d(0, 0, 0),
                        [](const cv::Vec3d& acc, const cv::Point3d& p)
                        {
                            return acc + cv::Vec3d(p.x, p.y, p.z);
                        }) /
                    gsl::narrow<double>(points3d_0.size());

    auto mean_dst = std::accumulate(
                        points3d_1.begin(),
                        points3d_1.end(),
                        cv::Vec3d(0, 0, 0),
                        [](const cv::Vec3d& acc, const cv::Point3d& p)
                        {
                            return acc + cv::Vec3d(p.x, p.y, p.z);
                        }) /
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
        for (auto c = 0; c < 3; ++c)
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

auto zenslam::utils::estimate_rigid_ransac(
    const std::vector<cv::Point3d>& src,
    const std::vector<cv::Point3d>& dst,
    cv::Matx33d&                    best_R,
    cv::Point3d&                    best_t,
    std::vector<size_t>&            inlier_indices,
    std::vector<size_t>&            outlier_indices,
    std::vector<double>&            errors,
    const double                    threshold,
    const int                       max_iterations) -> bool
{
    if (src.size() != dst.size() || src.size() < 3)
        return false;

    std::mt19937                          rng { std::random_device {}() };
    std::uniform_int_distribution<size_t> dist(0, src.size() - 1);

    auto                best_inlier_count = 0;
    std::vector<size_t> best_inliers;
    cv::Matx33d         bestR;
    cv::Vec3d           bestt;
    std::vector<double> best_errors {};

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
            cv::Vec3d  p   = R * cv::Point3d(src[i].x, src[i].y, src[i].z) + t;
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

void zenslam::utils::umeyama(
    const std::vector<cv::Point3d>& src,
    const std::vector<cv::Point3d>& dst,
    cv::Matx33d&                    R,
    cv::Point3d&                    t)
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
        auto a      = cv::Vec3d(src[i].x, src[i].y, src[i].z) - mean_src;
        auto b      = cv::Vec3d(dst[i].x, dst[i].y, dst[i].z) - mean_dst;
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
        for (auto c = 0; c < 3; ++c)
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
