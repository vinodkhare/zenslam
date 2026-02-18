#pragma once

#include <vector>

#include <opencv2/core/types.hpp>

#include "zenslam/options.h"
#include "zenslam/types/keyline.h"
#include "zenslam/types/line3d.h"

namespace zenslam::utils
{
    /**
     * Triangulate keylines between stereo frames using their indices.
     * For each keyline index present in both maps, triangulate the two endpoints;
     * the returned @ref zenslam::line3d stores just the 3D endpoints (no separate midpoint).
     *
     * @param keylines_0 Map of keylines in left image
     * @param keylines_1 Map of keylines in right image
     * @param projection_0 3x4 projection matrix for left camera
     * @param projection_1 3x4 projection matrix for right camera
     * @param options SLAM options containing triangulation parameters
     * @param translation_of_camera1_in_camera0 Translation vector between cameras
     * @return Vector of 3D line segments with original keyline indices
     */
    auto triangulate_keylines(
        const map<keyline>& keylines_0,
        const map<keyline>& keylines_1,
        const cv::Matx34d&  projection_0,
        const cv::Matx34d&  projection_1,
        const slam_options& options,
        const cv::Vec3d&    translation_of_camera1_in_camera0) -> std::vector<line3d>;

    /** Triangulate 3D points from matched 2D keypoints in stereo images.
     *
     * This function triangulates 3D points from corresponding 2D keypoints in left and right images.
     * It uses the provided projection matrices for the left and right cameras to perform triangulation.
     *
     * @param points2f_0 Vector of 2D keypoints in the left image.
     * @param points2f_1 Vector of 2D keypoints in the right image.
     * @param projection_0 The 3x4 projection matrix for the left camera.
     * @param projection_1 The 3x4 projection matrix for the right camera.
     * @return A vector of triangulated 3D points.
     */
    auto triangulate_points(
        const std::vector<cv::Point2f>& points2f_0,
        const std::vector<cv::Point2f>& points2f_1,
        const cv::Matx34d&              projection_0,
        const cv::Matx34d&              projection_1) -> std::vector<cv::Point3d>;
}
