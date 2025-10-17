#pragma once

#include <map>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "options.h"
#include "point.h"
#include "stereo_frame.h"

namespace zenslam::utils
{
    auto draw_keypoints(const camera_frame &frame) -> cv::Mat;
    auto draw_matches(const stereo_frame &frame, const std::map<size_t, point> &points) -> cv::Mat;
    auto draw_matches(const camera_frame &frame_0, const camera_frame &frame_1) -> cv::Mat;
    auto project(const std::vector<cv::Point3d> &points, const cv::Matx34d &projection) -> std::vector<cv::Point2d>;
    auto pyramid(const cv::Mat &image, const class options::slam &options) -> std::vector<cv::Mat>;
}
