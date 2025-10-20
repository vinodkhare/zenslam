#include "utils_opencv.h"

#include <random>

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

void zenslam::utils::draw_line_matches
(
    const cv::Mat&                                   img1,
    const std::vector<cv::line_descriptor::KeyLine>& keylines1,
    const cv::Mat&                                   img2,
    const std::vector<cv::line_descriptor::KeyLine>& keylines2,
    const std::vector<cv::DMatch>&                   matches1to2,
    cv::Mat&                                         outImg,
    const cv::Scalar&                                matchColor,
    const cv::Scalar&                                singleLineColor,
    const std::vector<char>&                         matchesMask,
    int                                              flags
)
{
    if (img1.type() != img2.type())
    {
        std::cout << "Input images have different types" << std::endl;
        CV_Assert(img1.type() == img2.type());
    }

    /* initialize output matrix (if necessary) */
    if (flags == cv::line_descriptor::DrawLinesMatchesFlags::DEFAULT)
    {
        /* check how many rows are necessary for output matrix */
        auto totalRows = img1.rows >= img2.rows ? img1.rows : img2.rows;

        /* initialize output matrix */
        outImg = cv::Mat::zeros(totalRows, img1.cols + img2.cols, img1.type());

        cv::Mat roi_left(outImg, cv::Rect(0, 0, img1.cols, img1.rows));
        cv::Mat roi_right(outImg, cv::Rect(img1.cols, 0, img2.cols, img2.rows));
        img1.copyTo(roi_left);
        img2.copyTo(roi_right);
    }
    else {}

    /* initialize random seed: */
    thread_local std::mt19937_64  rng { std::random_device { }() };
    std::uniform_int_distribution dist(0, 255);

    cv::Scalar singleLineColorRGB;
    if (singleLineColor == cv::Scalar::all(-1))
    {
        auto R = dist(rng);
        auto G = dist(rng);
        auto B = dist(rng);

        singleLineColorRGB = cv::Scalar(R, G, B);
    }
    else singleLineColorRGB = singleLineColor;

    /* get columns offset */
    auto offset = img1.cols;

    /* if requested, draw lines from both images */
    if (flags != cv::line_descriptor::DrawLinesMatchesFlags::NOT_DRAW_SINGLE_LINES)
    {
        for (auto k1: keylines1)
        {
            //line( outImg, Point2f( k1.startPointX, k1.startPointY ), Point2f( k1.endPointX, k1.endPointY ), singleLineColorRGB, 2 );
            line(outImg, cv::Point2f(k1.startPointX, k1.startPointY), cv::Point2f(k1.endPointX, k1.endPointY), singleLineColorRGB, 2);
        }

        for (auto k2: keylines2)
        {
            line
            (
                outImg,
                cv::Point2f(k2.startPointX + gsl::narrow<float>(offset), k2.startPointY),
                cv::Point2f(k2.endPointX + gsl::narrow<float>(offset), k2.endPointY),
                singleLineColorRGB,
                2
            );
        }
    }

    /* draw matches */
    for (size_t counter = 0; counter < matches1to2.size(); counter++)
    {
        if (matchesMask[counter] != 0)
        {
            auto dm    = matches1to2[counter];
            auto left  = keylines1[dm.queryIdx];
            auto right = keylines2[dm.trainIdx];

            cv::Scalar matchColorRGB;
            if (matchColor == cv::Scalar::all(-1))
            {
                auto R = dist(rng);
                auto G = dist(rng);
                auto B = dist(rng);

                matchColorRGB = cv::Scalar(R, G, B);

                if (singleLineColor == cv::Scalar::all(-1)) singleLineColorRGB = matchColorRGB;
            }

            else matchColorRGB = matchColor;

            /* draw lines if necessary */
            //      line( outImg, Point2f( left.startPointX, left.startPointY ), Point2f( left.endPointX, left.endPointY ), singleLineColorRGB, 2 );
            //
            //      line( outImg, Point2f( right.startPointX + offset, right.startPointY ), Point2f( right.endPointX + offset, right.endPointY ), singleLineColorRGB,
            //            2 );
            //
            //      /* link correspondent lines */
            //      line( outImg, Point2f( left.startPointX, left.startPointY ), Point2f( right.startPointX + offset, right.startPointY ), matchColorRGB, 1 );

            line
            (
                outImg,
                cv::Point2f(left.startPointX, left.startPointY),
                cv::Point2f(left.endPointX, left.endPointY),
                singleLineColorRGB,
                2
            );

            line
            (
                outImg,
                cv::Point2f(right.startPointX + gsl::narrow<float>(offset), right.startPointY),
                cv::Point2f(right.endPointX + gsl::narrow<float>(offset), right.endPointY),
                singleLineColorRGB,
                2
            );

            /* link correspondent lines */
            line
            (
                outImg,
                left.pt,
                right.pt + cv::Point2f(gsl::narrow<float>(offset), 0.0f),
                matchColorRGB,
                1
            );
        }
    }
}

auto zenslam::utils::draw_matches_spatial(const frame::stereo& frame, const map<point3d>& points) -> cv::Mat
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

auto zenslam::utils::draw_matches_temporal(const frame::camera& frame_0, const frame::camera& frame_1, bool show_keypoints, bool show_keylines) -> cv::Mat
{
    // Prepare images with keylines
    auto image_0 = frame_0.undistorted.clone();
    auto image_1 = frame_1.undistorted.clone();

    if (image_0.channels() == 1) image_0 = convert_color(image_0, cv::COLOR_GRAY2BGR);
    if (image_1.channels() == 1) image_1 = convert_color(image_1, cv::COLOR_GRAY2BGR);

    // Draw keypoints if enabled
    if (show_keypoints)
    {
        const auto& keypoints_0 = frame_0.keypoints.values() | std::ranges::to<std::vector>();
        const auto& keypoints_1 = frame_1.keypoints.values() | std::ranges::to<std::vector>();

        cv::drawKeypoints
        (
            image_0,
            utils::cast<cv::KeyPoint>(keypoints_0),
            image_0,
            cv::viz::Color::raspberry(),
            cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS | cv::DrawMatchesFlags::DRAW_OVER_OUTIMG
        );

        cv::drawKeypoints
        (
            image_1,
            utils::cast<cv::KeyPoint>(keypoints_1),
            image_1,
            cv::viz::Color::raspberry(),
            cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS | cv::DrawMatchesFlags::DRAW_OVER_OUTIMG
        );
    }

    // Draw keylines if enabled
    if (show_keylines)
    {
        const auto& keylines_0 = frame_0.keylines.values() | std::ranges::to<std::vector>();
        const auto& keylines_1 = frame_1.keylines.values() | std::ranges::to<std::vector>();

        cv::line_descriptor::drawKeylines
        (
            image_0,
            utils::cast<cv::line_descriptor::KeyLine>(keylines_0),
            image_0,
            cv::viz::Color::raspberry(),
            cv::line_descriptor::DrawLinesMatchesFlags::DRAW_OVER_OUTIMG
        );

        cv::line_descriptor::drawKeylines
        (
            image_1,
            utils::cast<cv::line_descriptor::KeyLine>(keylines_1),
            image_1,
            cv::viz::Color::raspberry(),
            cv::line_descriptor::DrawLinesMatchesFlags::DRAW_OVER_OUTIMG
        );
    }

    cv::Mat matches_image { };
    cv::hconcat(image_0, image_1, matches_image);

    // Draw keypoint matches if enabled
    if (show_keypoints)
    {
        const auto& keypoints_0_matched = frame_0.keypoints.values_matched(frame_1.keypoints) | std::ranges::to<std::vector>();
        const auto& keypoints_1_matched = frame_1.keypoints.values_matched(frame_0.keypoints) | std::ranges::to<std::vector>();
        const auto& matches             = utils::matches(keypoints_0_matched.size());

        cv::drawMatches
        (
            image_0,
            utils::cast<cv::KeyPoint>(keypoints_0_matched),
            image_1,
            utils::cast<cv::KeyPoint>(keypoints_1_matched),
            matches,
            matches_image,
            cv::viz::Color::green(),
            cv::viz::Color::blue(),
            std::vector<char>(),
            cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS | cv::DrawMatchesFlags::DRAW_OVER_OUTIMG
        );
    }

    // Draw keyline matches if enabled
    if (show_keylines)
    {
        const auto& keylines_0_matched = frame_0.keylines.values_matched(frame_1.keylines) | std::ranges::to<std::vector>();
        const auto& keylines_1_matched = frame_1.keylines.values_matched(frame_0.keylines) | std::ranges::to<std::vector>();
        const auto& line_matches       = utils::matches(keylines_0_matched.size());

        draw_line_matches
        (
            image_0,
            utils::cast<cv::line_descriptor::KeyLine>(keylines_0_matched),
            image_1,
            utils::cast<cv::line_descriptor::KeyLine>(keylines_1_matched),
            line_matches,
            matches_image,
            cv::viz::Color::all(-1),
            cv::viz::Color::all(-1),
            std::vector<char>(line_matches.size(), true),
            cv::line_descriptor::DrawLinesMatchesFlags::DRAW_OVER_OUTIMG
        );
    }

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
