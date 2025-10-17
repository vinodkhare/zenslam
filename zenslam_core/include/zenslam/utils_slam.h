#pragma once

#include <map>

#include <opencv2/imgproc.hpp>

#include "camera_calibration.h"
#include "keypoint.h"
#include "options.h"
#include "point.h"
#include "pose_data.h"
#include "stereo_frame.h"


inline auto operator+=
(
    std::map<size_t, zenslam::keypoint> & keypoints_map,
    const std::vector<zenslam::keypoint> &keypoints
) -> std::map<size_t, zenslam::keypoint> &
{
    for (const auto &keypoint: keypoints)
    {
        keypoints_map[keypoint.index] = keypoint;
    }
    return keypoints_map;
}


namespace zenslam::utils
{
    auto correspondences_3d2d
    (
        const std::map<size_t, point> &   points,
        const std::map<size_t, keypoint> &keypoints,
        std::vector<cv::Point3d> &        points3d,
        std::vector<cv::Point2d> &        points2d,
        std::vector<size_t> &             indices
    ) -> void;

    void correspondences_3d3d
    (
        const std::map<size_t, point> &points_map_0,
        const std::map<size_t, point> &points_map_1,
        std::vector<cv::Point3d> &     points3d_0,
        std::vector<cv::Point3d> &     points3d_1,
        std::vector<size_t> &          indexes
    );

    auto estimate_pose_3d2d
    (
        const std::map<size_t, point> &   map_points_0,
        const std::map<size_t, keypoint> &map_keypoints_1,
        const cv::Matx33d &               camera_matrix,
        const double &                    threshold
    ) -> pose_data;

    auto estimate_pose_3d3d
    (
        const std::map<size_t, point> &map_points_0,
        const std::map<size_t, point> &map_points_1,
        const double &                 threshold
    ) -> pose_data;

    // Estimate rigid transform (rotation R and translation t) between two sets of 3D points
    // src, dst: corresponding points
    // Returns true if successful, false otherwise
    auto estimate_rigid
    (
        const std::vector<cv::Point3d> &src,
        const std::vector<cv::Point3d> &dst,
        cv::Matx33d &                   R,
        cv::Point3d &                   t
    ) -> bool;

    // RANSAC wrapper for estimate_rigid: returns best R, t, and inlier/outlier indices
    auto estimate_rigid_ransac
    (
        const std::vector<cv::Point3d> &src,
        const std::vector<cv::Point3d> &dst,
        cv::Matx33d &                   best_R,
        cv::Point3d &                   best_t,
        std::vector<size_t> &           inlier_indices,
        std::vector<size_t> &           outlier_indices,
        std::vector<double> &           errors,
        double                          threshold      = 0.01,
        int                             max_iterations = 1000
    ) -> bool;

    // filters matches using the epipolar crterion given the fundamental matrix
    auto filter
    (
        const std::vector<cv::KeyPoint> &keypoints0,
        const std::vector<cv::KeyPoint> &keypoints1,
        const std::vector<cv::DMatch> &  matches,
        const cv::Matx33d &              fundamental,
        double                           epipolar_threshold
    ) -> std::vector<cv::DMatch>;

    auto match
    (
        const std::map<size_t, keypoint> &map_keypoints_l,
        const std::map<size_t, keypoint> &map_keypoints_r,
        const cv::Matx33d &               fundamental,
        double                            epipolar_threshold
    ) -> std::vector<cv::DMatch>;

    auto match_temporal
    (
        const std::map<size_t, keypoint> &keypoints_map_0,
        const std::map<size_t, keypoint> &keypoints_map_1,
        const cv::Matx33d &               camera_matrix,
        double                            threshold
    ) -> std::vector<cv::DMatch>;

    /** Pre-process a camera frame by converting to grayscale, applying CLAHE, and building an image pyramid.
     *
     * @param frame The input camera frame containing the image to be processed.
     * @param calibration The camera calibration parameters.
     * @param options SLAM options that may include CLAHE settings and pyramid levels.
     * @param clahe A pointer to an OpenCV CLAHE object configured with desired parameters.
     * @return The pre-processed frame.
     */
    auto pre_process
    (
        const camera_frame &       frame,
        const camera_calibration & calibration,
        const class options::slam &options,
        const cv::Ptr<cv::CLAHE> & clahe
    ) -> camera_frame;

    /** Pre-process a camera frame by converting to grayscale, applying CLAHE, and building an image pyramid.
     *
     * @param frame The input stereo frame containing the image to be processed.
     * @param calibration The camera calibration parameters.
     * @param options SLAM options that may include CLAHE settings and pyramid levels.
     * @return The pre-processed frame.
     */
    auto pre_process
    (
        const stereo_frame &                     frame,
        const std::array<camera_calibration, 2> &calibration,
        const class options::slam &              options,
        const cv::Ptr<cv::CLAHE> &               clahe
    ) -> stereo_frame;

    auto solve_pnp
    (
        const cv::Matx33d &             camera_matrix,
        const std::vector<cv::Point3d> &points3d,
        const std::vector<cv::Point2d> &points2d,
        cv::Affine3d &                  pose
    ) -> void;

    /** Track keypoints from frame_0 to frame_1 using KLT optical flow.
     *
     * @param pyramid_0 The image pyramid of the first frame.
     * @param pyramid_1 The image pyramid of the second frame.
     * @param keypoints_map_0 A map of keypoints in the first frame to be tracked.
     * @param options SLAM options that may include KLT parameters.
     * @param points_1_predicted Optional predicted positions of keypoints in frame_1 for improved tracking.
     * @return A vector of tracked keypoints in frame_1.
     */
    auto track
    (
        const std::vector<cv::Mat> &      pyramid_0,
        const std::vector<cv::Mat> &      pyramid_1,
        const std::map<size_t, keypoint> &keypoints_map_0,
        const class options::slam &       options,
        const std::vector<cv::Point2f> &  points_1_predicted = {}
    ) -> std::vector<keypoint>;

    /** Track keypoints between two stereo frames.
     *
     * @param frames An array containing the two stereo frames to track between.
     * @param options SLAM options that may include KLT parameters.
     * @return An array of stereo frames with tracked keypoints.
     */
    auto track
    (
        const std::array<stereo_frame, 2> &frames,
        const class options::slam &        options
    ) -> std::array<std::vector<keypoint>, 2>;

    auto triangulate
    (
        stereo_frame &     frame,
        const cv::Matx34d &projection_l,
        const cv::Matx34d &projection_r
    ) -> std::tuple<std::map<size_t, point>, std::vector<double>>;

    auto umeyama
    (
        const std::vector<cv::Point3d> &src,
        const std::vector<cv::Point3d> &dst,
        cv::Matx33d &                   R,
        cv::Point3d &                   t
    ) -> void;

    auto undistort(const cv::Mat &image, const camera_calibration &calibration) -> cv::Mat;
}
