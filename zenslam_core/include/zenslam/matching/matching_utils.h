#pragma once

#include <map>
#include <vector>

#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>

#include "zenslam/all_options.h"
#include "zenslam/all_options.h"
#include "zenslam/types/keyline.h"
#include "zenslam/types/keypoint.h"
#include "zenslam/types/point3d_cloud.h"

namespace zenslam::utils
{
    /**
     * Filters matches using the epipolar criterion given the fundamental matrix
     * @param keypoints0 Keypoints from first image
     * @param keypoints1 Keypoints from second image
     * @param matches Initial matches
     * @param fundamental Fundamental matrix
     * @param epipolar_threshold Maximum epipolar distance for inliers
     * @return Filtered matches
     */
    auto filter(
        const std::vector<cv::KeyPoint>& keypoints0,
        const std::vector<cv::KeyPoint>& keypoints1,
        const std::vector<cv::DMatch>&   matches,
        const cv::Matx33d&               fundamental,
        double                           epipolar_threshold) -> std::vector<cv::DMatch>;

    /**
     * Create a descriptor matcher based on options.
     * Creates the appropriate matcher (BFMatcher or FlannBasedMatcher) depending on
     * matcher type and descriptor type (binary vs float).
     *
     * @param options SLAM options containing matcher configuration
     * @param is_binary True for binary descriptors (ORB, BRISK), false for float (SIFT, SURF)
     * @return Pointer to the created matcher
     */
    auto create_matcher(
        const slam_options& options,
        bool                is_binary) -> cv::Ptr<cv::DescriptorMatcher>;

    /**
     * Match 3D points to 2D keypoints by projecting the 3D points into the camera frame
     * and finding keypoints within a specified radius.
     *
     * @param points3d_world Map of 3D points in the world coordinate system
     * @param keypoints Map of 2D keypoints in the image
     * @param pose_of_camera0_in_world Affine transformation representing the camera pose in the world
     * @param projection Projection matrix
     * @param radius Radius (in meters) for matching projected 3D points to 2D keypoints
     * @param threshold Reprojection error threshold
     * @return Vector of cv::DMatch representing the matched 3D-2D correspondences
     */
    auto match_keypoints3d(
        const point3d_cloud& points3d_world,
        const map<keypoint>& keypoints,
        const cv::Affine3d&  pose_of_camera0_in_world,
        const cv::Matx34d&   projection,
        double               radius,
        double               threshold) -> std::vector<cv::DMatch>;

    /**
     * Match 3D points to 2D keypoints with full frustum culling support.
     * This overload uses slam_options for configuration and performs frustum culling
     * based on image dimensions and configured margin.
     *
     * @param points3d_world Map of 3D points in the world coordinate system
     * @param keypoints Map of 2D keypoints in the image
     * @param pose_of_camera0_in_world Affine transformation representing the camera pose in the world
     * @param projection Projection matrix
     * @param image_size Size of the image for frustum culling
     * @param radius Radius (in meters) for matching projected 3D points to 2D keypoints
     * @param options SLAM options containing reprojection and frustum culling parameters
     * @return Vector of cv::DMatch representing the matched 3D-2D correspondences
     */
    auto match_keypoints3d(
        const point3d_cloud& points3d_world,
        const map<keypoint>& keypoints,
        const cv::Affine3d&  pose_of_camera0_in_world,
        const cv::Matx34d&   projection,
        const cv::Size&      image_size,
        double               radius,
        const slam_options&  options) -> std::vector<cv::DMatch>;

    /**
     * Match keylines between two frames using their descriptors, then filter matches using epipolar constraint
     * on both endpoints and the midpoint.
     *
     * @param keylines_map_0 Keylines from frame 0
     * @param keylines_map_1 Keylines from frame 1
     * @param fundamental Fundamental matrix between the two frames
     * @param epipolar_threshold Threshold for epipolar error (pixels)
     * @return Vector of cv::DMatch for good matches
     */
    auto match_keylines(
        const map<keyline>& keylines_map_0,
        const map<keyline>& keylines_map_1,
        const cv::Matx33d&  fundamental,
        double              epipolar_threshold) -> std::vector<cv::DMatch>;

    /**
     * Perform temporal matching between consecutive frames using essential matrix constraint
     *
     * @param keypoints_map_0 Keypoints from frame 0
     * @param keypoints_map_1 Keypoints from frame 1
     * @param camera_matrix Camera intrinsic matrix
     * @param threshold Reprojection error threshold
     * @return Vector of matches
     */
    auto match_temporal(
        const std::map<size_t, keypoint>& keypoints_map_0,
        const std::map<size_t, keypoint>& keypoints_map_1,
        const cv::Matx33d&                camera_matrix,
        double                            threshold) -> std::vector<cv::DMatch>;

    /** Solve the PnP problem to estimate camera pose from 3D-2D point correspondences.
     *
     * @param camera_matrix The intrinsic camera matrix.
     * @param points3d A vector of 3D points in the world coordinate system.
     * @param points2d A vector of corresponding 2D points in the image plane.
     * @param pose Output affine transformation representing the camera pose.
     */
    auto solve_pnp(
        const cv::Matx33d&              camera_matrix,
        const std::vector<cv::Point3d>& points3d,
        const std::vector<cv::Point2d>& points2d,
        cv::Affine3d&                   pose) -> void;
}
