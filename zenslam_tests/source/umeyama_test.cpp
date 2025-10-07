#include <catch2/catch_all.hpp>

#include <cmath>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include "zenslam/utils.h"

TEST_CASE("umeyama estimates rotation and translation for rigid transform")
{
    using namespace zenslam;

    // Create a simple set of 4 non-coplanar points
    std::vector<cv::Point3d> src = {
        { 0.0, 0.0, 0.0 },
        { 1.0, 0.0, 0.0 },
        { 0.0, 1.0, 0.0 },
        { 0.0, 0.0, 1.0 }
    };

    // Ground-truth rotation: 90 deg around Z axis
    cv::Matx33d     R_gt;
    const cv::Vec3d rvec_gt { M_PI / 2.0, 0, 0 };
    cv::Rodrigues(rvec_gt, R_gt);

    // Ground-truth translation
    cv::Vec3d t_gt(1.5, -2.0, 0.5);

    std::vector<cv::Point3d> dst;
    dst.reserve(src.size());
    for (const auto &p: src)
    {
        cv::Vec3d pv(p.x, p.y, p.z);
        auto      q = R_gt * pv + t_gt;
        dst.emplace_back(q[0], q[1], q[2]);
    }

    cv::Matx33d R_est;
    cv::Point3d   t_est;

    utils::umeyama(src, dst, R_est, t_est);

    // Compare rotation matrices and translation with tolerance
    constexpr auto tol = 1e-6;

    for (auto r = 0; r < 3; ++r)
    {
        for (auto c = 0; c < 3; ++c)
        {
            REQUIRE(std::abs(R_est(r, c) - R_gt(r, c)) < tol);
        }
    }
}
