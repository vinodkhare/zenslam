#include "utils_opencv.h"

#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include "utils_std.h"

#include <gsl/narrow>

auto zenslam::utils::draw_keypoints(const mono_frame &frame) -> cv::Mat
{
    const auto &keypoints = utils::cast<cv::KeyPoint>(values(frame.keypoints));

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

auto zenslam::utils::draw_matches(const stereo_frame &frame, const std::map<size_t, point> &points) -> cv::Mat
{
    cv::Mat matches_image { };

    const auto &keypoints_l = values(frame.l.keypoints);
    const auto &keypoints_r = values(frame.r.keypoints);

    // Create base image by concatenating left and right images
    cv::hconcat(frame.l.undistorted, frame.r.undistorted, matches_image);
    cv::cvtColor(matches_image, matches_image, cv::COLOR_GRAY2BGR);

    // Track which keypoints are matched
    std::vector<bool> l_matched(keypoints_l.size(), false);
    std::vector<bool> r_matched(keypoints_r.size(), false);

    // Separate matches into triangulated and not triangulated
    std::vector<cv::DMatch> triangulated_matches { };
    std::vector<cv::DMatch> matched_not_triangulated { };

    for (auto query = 0; query < keypoints_l.size(); query++)
    {
        for (auto train = 0; train < keypoints_r.size(); train++)
        {
            if (keypoints_l[query].index == keypoints_r[train].index)
            {
                l_matched[query] = true;
                r_matched[train] = true;

                // Check if this keypoint has been triangulated
                if (points.contains(keypoints_l[query].index))
                {
                    triangulated_matches.emplace_back(query, train, 1.0);
                }
                else
                {
                    matched_not_triangulated.emplace_back(query, train, 1.0);
                }
            }
        }
    }

    // Draw unmatched keypoints in red
    for (auto i = 0; i < keypoints_l.size(); i++)
    {
        if (!l_matched[i])
        {
            const auto &kp = static_cast<cv::KeyPoint>(keypoints_l[i]);
            cv::circle(matches_image, kp.pt, gsl::narrow<int>(kp.size * 2), cv::Scalar(000, 000, 255), 2);
        }
    }
    for (auto i = 0; i < keypoints_r.size(); i++)
    {
        if (!r_matched[i])
        {
            const auto &kp = static_cast<cv::KeyPoint>(keypoints_r[i]);
            cv::Point2f pt_shifted = kp.pt;
            pt_shifted.x += static_cast<float>(frame.l.undistorted.cols);
            cv::circle(matches_image, pt_shifted, gsl::narrow<int>(kp.size * 2), cv::Scalar(000, 000, 255), 2);
        }
    }

    // Draw matched but not triangulated in yellowish-orange
    for (const auto &match : matched_not_triangulated)
    {
        const auto &kp_l = static_cast<cv::KeyPoint>(keypoints_l[match.queryIdx]);
        const auto &kp_r = static_cast<cv::KeyPoint>(keypoints_r[match.trainIdx]);
        
        cv::Point2f pt_r_shifted = kp_r.pt;
        pt_r_shifted.x += static_cast<float>(frame.l.undistorted.cols);
        
        cv::line(matches_image, kp_l.pt, pt_r_shifted, cv::Scalar(000, 165, 255), 1);
        cv::circle(matches_image, kp_l.pt, gsl::narrow<int>(kp_l.size * 2), cv::Scalar(000, 165, 255), 2);
        cv::circle(matches_image, pt_r_shifted, gsl::narrow<int>(kp_r.size * 2), cv::Scalar(000, 165, 255), 2);
    }

    // Draw triangulated matches in green
    for (const auto &match : triangulated_matches)
    {
        const auto &kp_l = static_cast<cv::KeyPoint>(keypoints_l[match.queryIdx]);
        const auto &kp_r = static_cast<cv::KeyPoint>(keypoints_r[match.trainIdx]);
        
        cv::Point2f pt_r_shifted = kp_r.pt;
        pt_r_shifted.x += static_cast<float>(frame.l.undistorted.cols);
        
        cv::line(matches_image, kp_l.pt, pt_r_shifted, cv::Scalar(000, 255, 000), 1);
        cv::circle(matches_image, kp_l.pt, gsl::narrow<int>(kp_l.size * 2), cv::Scalar(000, 255, 000), 2);
        cv::circle(matches_image, pt_r_shifted, gsl::narrow<int>(kp_r.size * 2), cv::Scalar(000, 255, 000), 2);
    }

    return matches_image;
}

auto zenslam::utils::draw_matches(const mono_frame &frame_0, const mono_frame &frame_1) -> cv::Mat
{
    cv::Mat matches_image { };

    const auto &keypoints_0 = values(frame_0.keypoints);
    const auto &keypoints_1 = values(frame_1.keypoints);

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

auto zenslam::utils::project(const std::vector<cv::Point3d> &points, const cv::Matx34d &projection) -> std::vector<cv::Point2d>
{
    std::vector<cv::Point2d> points2d { };
    cv::Mat                  points3d_mat(4, gsl::narrow<int>(points.size()), CV_64F);

    for (int i = 0; i < points.size(); i++)
    {
        points3d_mat.at<double>(0, i) = points[i].x;
        points3d_mat.at<double>(1, i) = points[i].y;
        points3d_mat.at<double>(2, i) = points[i].z;
        points3d_mat.at<double>(3, i) = 1.0;
    }

    cv::Mat points2d_mat = projection * points3d_mat;

    points2d = std::views::iota(0, gsl::narrow<int>(points.size())) | std::views::transform
               (
                   [&points2d_mat](const auto &i)
                   {
                       return std::abs(points2d_mat.at<double>(2, i)) > 1E-9
                                  ? cv::Point2d
                                  (
                                      points2d_mat.at<double>(0, i) / points2d_mat.at<double>(2, i),
                                      points2d_mat.at<double>(1, i) / points2d_mat.at<double>(2, i)
                                  )
                                  : cv::Point2d(0.0, 0.0);
                   }
               ) | std::ranges::to<std::vector>();

    for (int i = 0; i < points.size(); i++)
    {
        double w = points2d_mat.at<double>(2, i);
        if (std::abs(w) > 1E-9)
        {
            points2d.emplace_back
            (
                points2d_mat.at<double>(0, i) / w,
                points2d_mat.at<double>(1, i) / w
            );
        }
        else
        {
            points2d.emplace_back(0.0, 0.0);
        }
    }

    return points2d;
}

auto zenslam::utils::pyramid(const cv::Mat &image, const class options::slam &options) -> std::vector<cv::Mat>
{
    std::vector<cv::Mat> pyramid { };
    cv::buildOpticalFlowPyramid(image, pyramid, options.klt_window_size, options.klt_max_level);
    return pyramid;
}
