#pragma once

#include <map>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <opencv2/core/mat.hpp>

#include "options.h"
#include "point.h"
#include "frame/stereo.h"

namespace zenslam::utils
{
    /** Apply CLAHE (Contrast Limited Adaptive Histogram Equalization) to an image using OpenCV.
     *
     * @param image The input image to be processed.
     * @param clahe A pointer to an OpenCV CLAHE object configured with desired parameters.
     * @return The image after applying CLAHE.
     */
    auto apply_clahe(const cv::Mat &image, const cv::Ptr<cv::CLAHE> &clahe) -> cv::Mat;

    /** Convert the color space of an image using OpenCV.
     *
     * @param image The input image to be converted.
     * @param code The OpenCV color conversion code (e.g., cv::COLOR_BGR2GRAY).
     * @return The color-converted image.
     */
    auto convert_color(const cv::Mat &image, int code) -> cv::Mat;

    auto draw_keypoints(const zenslam::frame::camera &frame) -> cv::Mat;
    auto draw_matches(const zenslam::frame::stereo &frame, const std::map<size_t, point> &points) -> cv::Mat;
    auto draw_matches(const zenslam::frame::camera &frame_0, const zenslam::frame::camera &frame_1) -> cv::Mat;
    auto project(const std::vector<cv::Point3d> &points, const cv::Matx34d &projection) -> std::vector<cv::Point2d>;
    auto pyramid(const cv::Mat &image, const class options::slam &options) -> std::vector<cv::Mat>;
}
