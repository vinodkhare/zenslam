#pragma once

#include <vector>

#include <opencv2/core/mat.hpp>

#include "options.h"
#include "stereo_frame.h"

namespace zenslam::utils
{
    auto draw_keypoints(const mono_frame &frame) -> cv::Mat;
    auto draw_matches(const stereo_frame &frame) -> cv::Mat;
    auto draw_matches(const mono_frame &frame_0, const mono_frame &frame_1) -> cv::Mat;
    auto project(const std::vector<cv::Point3d> &points, const cv::Matx34d &projection) -> std::vector<cv::Point2d>;
    auto pyramid(const cv::Mat &image, const class options::slam &options) -> std::vector<cv::Mat>;
}
