#include "utils_opencv.h"

#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include "utils_std.h"

#include <gsl/narrow>

auto zenslam::utils::apply_clahe(const cv::Mat &image, const cv::Ptr<cv::CLAHE> &clahe) -> cv::Mat
{
    cv::Mat converted_image { };
    clahe->apply(image, converted_image);
    return converted_image;
}

auto zenslam::utils::convert_color(const cv::Mat &image, int code) -> cv::Mat
{
    cv::Mat converted_image { };
    cv::cvtColor(image, converted_image, code);
    return converted_image;
}

auto zenslam::utils::draw_keypoints(const zenslam::frame::camera &frame) -> cv::Mat
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

auto zenslam::utils::draw_matches(const zenslam::frame::stereo &frame, const std::map<size_t, point> &points) -> cv::Mat
{
    cv::Mat matches_image { };

    const auto &keypoints_l = values(frame.cameras[0].keypoints);
    const auto &keypoints_r = values(frame.cameras[1].keypoints);

    const auto &keypoints_l_matched = std::views::filter
                                      (
                                          keypoints_l,
                                          [&frame](const auto &kp)
                                          {
                                              return frame.cameras[1].keypoints.contains(kp.index);
                                          }
                                      ) | std::ranges::to<std::vector>();
    const auto &keypoints_r_matched = std::views::filter
                                      (
                                          keypoints_r,
                                          [&frame](const auto &kp)
                                          {
                                              return frame.cameras[0].keypoints.contains(kp.index);
                                          }
                                      ) | std::ranges::to<std::vector>();

    const auto &keypoints_l_triangulated = std::views::filter
                                           (
                                               keypoints_l_matched,
                                               [&points](const auto &kp)
                                               {
                                                   return points.contains(kp.index);
                                               }
                                           ) | std::ranges::to<std::vector>();
    const auto &keypoints_r_triangulated = std::views::filter
                                           (
                                               keypoints_r_matched,
                                               [&points](const auto &kp)
                                               {
                                                   return points.contains(kp.index);
                                               }
                                           ) | std::ranges::to<std::vector>();

    auto undistorted_l = frame.cameras[0].undistorted.clone();
    auto undistorted_r = frame.cameras[1].undistorted.clone();

    const auto &matches = std::views::iota(0, gsl::narrow<int>(keypoints_l_matched.size())) | std::views::transform
                          (
                              [](auto i)
                              {
                                  return cv::DMatch(i, i, 0.0);
                              }
                          ) | std::ranges::to<std::vector>();
    const auto &matches_triangulated = std::views::iota
                                       (0, gsl::narrow<int>(keypoints_l_triangulated.size())) | std::views::transform
                                       (
                                           [](auto i)
                                           {
                                               return cv::DMatch(i, i, 0.0);
                                           }
                                       ) | std::ranges::to<std::vector>();

    cv::cvtColor(undistorted_l, undistorted_l, cv::COLOR_GRAY2BGR);
    cv::cvtColor(undistorted_r, undistorted_r, cv::COLOR_GRAY2BGR);

    cv::drawKeypoints
    (
        undistorted_l,
        utils::cast<cv::KeyPoint>(keypoints_l),
        undistorted_l,
        cv::Scalar(0, 0, 255),
        cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS | cv::DrawMatchesFlags::DRAW_OVER_OUTIMG
    );

    cv::drawKeypoints
    (
        undistorted_r,
        utils::cast<cv::KeyPoint>(keypoints_r),
        undistorted_r,
        cv::Scalar(0, 0, 255),
        cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS | cv::DrawMatchesFlags::DRAW_OVER_OUTIMG
    );

    cv::hconcat(undistorted_l, undistorted_r, matches_image);

    cv::drawMatches
    (
        undistorted_l,
        utils::cast<cv::KeyPoint>(keypoints_l_matched),
        undistorted_r,
        utils::cast<cv::KeyPoint>(keypoints_r_matched),
        matches,
        matches_image,
        cv::Scalar(0, 165, 255),
        cv::Scalar(255, 0, 0),
        std::vector<char>(),
        cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS | cv::DrawMatchesFlags::DRAW_OVER_OUTIMG
    );

    cv::drawMatches
    (
        undistorted_l,
        utils::cast<cv::KeyPoint>(keypoints_l_triangulated),
        undistorted_r,
        utils::cast<cv::KeyPoint>(keypoints_r_triangulated),
        matches_triangulated,
        matches_image,
        cv::Scalar(0, 255, 0),
        cv::Scalar(255, 0, 0),
        std::vector<char>(),
        cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS | cv::DrawMatchesFlags::DRAW_OVER_OUTIMG
    );

    return matches_image;
}

auto zenslam::utils::draw_matches(const zenslam::frame::camera &frame_0, const zenslam::frame::camera &frame_1) -> cv::Mat
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

    for (auto i = 0; i < points.size(); i++)
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

    for (auto i = 0; i < points.size(); i++)
    {
        auto w = points2d_mat.at<double>(2, i);
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
