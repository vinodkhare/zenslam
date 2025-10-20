#pragma once

#include <map>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <opencv2/core/mat.hpp>

#include "options.h"
#include "point3d.h"
#include "frame/stereo.h"

namespace zenslam::utils
{
    /** Apply CLAHE (Contrast Limited Adaptive Histogram Equalization) to an image using OpenCV.
     *
     * @param image The input image to be processed.
     * @param clahe A pointer to an OpenCV CLAHE object configured with desired parameters.
     * @return The image after applying CLAHE.
     */
    auto apply_clahe(const cv::Mat& image, const cv::Ptr<cv::CLAHE>& clahe) -> cv::Mat;

    /** Convert the color space of an image using OpenCV.
     *
     * @param image The input image to be converted.
     * @param code The OpenCV color conversion code (e.g., cv::COLOR_BGR2GRAY).
     * @return The color-converted image.
     */
    auto convert_color(const cv::Mat& image, int code) -> cv::Mat;

    /** Draw matches between line descriptors in two images.
     *
     * @param img1 First input image.
     * @param keylines1 Keylines detected in the first image.
     * @param img2 Second input image.
     * @param keylines2 Keylines detected in the second image.
     * @param matches1to2 Matches between keylines in the first and second images.
     * @param outImg Output image showing the matches.
     * @param matchColor Color for drawing matched lines.
     * @param singleLineColor Color for drawing unmatched lines.
     * @param matchesMask Mask to indicate which matches to draw.
     * @param flags Drawing flags (e.g., default behavior).
     */
    void draw_line_matches
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
    );

    auto draw_matches_spatial(const frame::stereo& frame, const map<point3d>& points) -> cv::Mat;
    auto draw_matches_temporal(const frame::camera& frame_0, const frame::camera& frame_1, bool show_keypoints = true, bool show_keylines = true) -> cv::Mat;
    auto project(const std::vector<cv::Point3d>& points, const cv::Matx34d& projection) -> std::vector<cv::Point2d>;
    auto pyramid(const cv::Mat& image, const class options::slam& options) -> std::vector<cv::Mat>;

    auto matches(size_t n) -> std::vector<cv::DMatch>;
}
