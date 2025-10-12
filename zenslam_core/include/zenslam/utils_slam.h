#pragma once

#include <map>

#include "calibration.h"
#include "keypoint.h"
#include "options.h"
#include "point.h"
#include "stereo_frame.h"

namespace zenslam::utils
{
    auto correspondences
    (
        const std::map<size_t, point> &   points,
        const std::map<size_t, keypoint> &keypoints,
        std::vector<cv::Point3d> &        points3d,
        std::vector<cv::Point2d> &        points2d
    ) -> void;

    void correspondences_3d3d
    (
        const std::map<size_t, point> &points_map_0,
        const std::map<size_t, point> &points_map_1,
        std::vector<cv::Point3d> &     points3d_0,
        std::vector<cv::Point3d> &     points3d_1,
        std::vector<size_t> &          indexes
    );

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
        int                             max_iterations = 1000,
        int                             min_inliers    = 3
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
        const std::map<size_t, keypoint> &keypoints_0,
        std::map<size_t, keypoint> &      keypoints_1,
        const cv::Matx33d &               fundamental,
        double                            epipolar_threshold
    ) -> void;

    auto solve_pnp
    (
        const cv::Matx33d &             camera_matrix,
        const std::vector<cv::Point3d> &points3d,
        const std::vector<cv::Point2d> &points2d,
        cv::Affine3d &                  pose
    ) -> void;

    auto track
    (
        const mono_frame &         frame_0,
        mono_frame &               frame_1,
        const class options::slam &options
    ) -> void;

    auto triangulate
    (
        stereo_frame &                  frame,
        const cv::Matx34d &             projection_l,
        const cv::Matx34d &             projection_r,
        std::map<unsigned long, point> &points
    ) -> void;

    auto umeyama
    (
        const std::vector<cv::Point3d> &src,
        const std::vector<cv::Point3d> &dst,
        cv::Matx33d &                   R,
        cv::Point3d &                   t
    ) -> void;

    auto undistort(const cv::Mat &image, const calibration &calibration) -> cv::Mat;
}
