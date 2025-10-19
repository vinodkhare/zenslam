#include "utils_opencv.h"

#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/line_descriptor.hpp>
#include <opencv2/video/tracking.hpp>

#include "utils_std.h"

#include <gsl/narrow>

auto zenslam::utils::apply_clahe(const cv::Mat& image, const cv::Ptr<cv::CLAHE>& clahe) -> cv::Mat
{
    cv::Mat converted_image { };
    clahe->apply(image, converted_image);
    return converted_image;
}

auto zenslam::utils::convert_color(const cv::Mat& image, int code) -> cv::Mat
{
    cv::Mat converted_image { };
    cv::cvtColor(image, converted_image, code);
    return converted_image;
}

auto zenslam::utils::draw_matches(const frame::stereo& frame, const map<point3d>& points) -> cv::Mat
{
    cv::Mat matches_image { };

    const auto& keypoints_l              = frame.cameras[0].keypoints.values() | std::ranges::to<std::vector>();
    const auto& keypoints_r              = frame.cameras[1].keypoints.values() | std::ranges::to<std::vector>();
    const auto& keypoints_l_matched      = frame.cameras[0].keypoints.values_matched(frame.cameras[1].keypoints) | std::ranges::to<std::vector>();
    const auto& keypoints_r_matched      = frame.cameras[1].keypoints.values_matched(frame.cameras[0].keypoints) | std::ranges::to<std::vector>();
    const auto& keypoints_l_triangulated = frame.cameras[0].keypoints.values_matched(frame.points3d) | std::ranges::to<std::vector>();
    const auto& keypoints_r_triangulated = frame.cameras[1].keypoints.values_matched(frame.points3d) | std::ranges::to<std::vector>();

    const auto& matches              = utils::matches(keypoints_l_matched.size());
    const auto& matches_triangulated = utils::matches(keypoints_l_triangulated.size());

    auto undistorted_l = frame.cameras[0].undistorted.clone();
    auto undistorted_r = frame.cameras[1].undistorted.clone();

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

auto zenslam::utils::draw_matches_temporal(const frame::camera& frame_0, const frame::camera& frame_1) -> cv::Mat
{
    // Prepare images with keylines
    auto image_0 = frame_0.undistorted.clone();
    auto image_1 = frame_1.undistorted.clone();

    if (image_0.channels() == 1) image_0 = convert_color(image_0, cv::COLOR_GRAY2BGR);
    if (image_1.channels() == 1) image_1 = convert_color(image_1, cv::COLOR_GRAY2BGR);

    const auto& keypoints_0 = frame_0.keypoints.values() | std::ranges::to<std::vector>();
    const auto& keypoints_1 = frame_1.keypoints.values() | std::ranges::to<std::vector>();

    cv::drawKeypoints(image_0, utils::cast<cv::KeyPoint>(keypoints_0), image_0, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::drawKeypoints(image_1, utils::cast<cv::KeyPoint>(keypoints_1), image_1, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    const auto& keypoints_0_matched = frame_0.keypoints.values_matched(frame_1.keypoints) | std::ranges::to<std::vector>();
    const auto& keypoints_1_matched = frame_1.keypoints.values_matched(frame_0.keypoints) | std::ranges::to<std::vector>();
    const auto& matches             = utils::matches(keypoints_0_matched.size());

    cv::Mat matches_image { };
    cv::drawMatches
    (
        image_0,
        utils::cast<cv::KeyPoint>(keypoints_0_matched),
        image_1,
        utils::cast<cv::KeyPoint>(keypoints_1_matched),
        matches,
        matches_image,
        cv::Scalar(000, 255, 000),
        cv::Scalar::all(-1),
        std::vector<char>(),
        cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS
    );

    return matches_image;
}

auto zenslam::utils::project(const std::vector<cv::Point3d>& points, const cv::Matx34d& projection) -> std::vector<cv::Point2d>
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
                   [&points2d_mat](const auto& i)
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
        const auto w = points2d_mat.at<double>(2, i);
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

auto zenslam::utils::pyramid(const cv::Mat& image, const class options::slam& options) -> std::vector<cv::Mat>
{
    std::vector<cv::Mat> pyramid { };
    cv::buildOpticalFlowPyramid(image, pyramid, options.klt_window_size, options.klt_max_level);
    return pyramid;
}

auto zenslam::utils::matches(size_t n) -> std::vector<cv::DMatch>
{
    return std::views::iota(static_cast<size_t>(0), n) | std::views::transform
           (
               [](auto i)
               {
                   return cv::DMatch(i, i, 0.0);
               }
           ) | std::ranges::to<std::vector>();
}
