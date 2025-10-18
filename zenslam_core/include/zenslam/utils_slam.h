#pragma once

#include <map>
#include <ranges>

#include <opencv2/imgproc.hpp>

#include "camera_calibration.h"
#include "keypoint.h"
#include "line3d.h"
#include "options.h"
#include "point3d.h"
#include "pose_data.h"
#include "frame/stereo.h"

inline auto operator-(const std::vector<cv::Point2d> &lhs, const std::vector<cv::Point2f> &rhs) -> std::vector<cv::Point2d>
{
    std::vector<cv::Point2d> difference;
    difference.reserve(lhs.size());

    for (auto i = 0; i < lhs.size(); ++i)
    {
        difference.emplace_back(lhs[i] - cv::Point2d(rhs[i].x, rhs[i].y));
    }

    return difference;
}

namespace zenslam::utils
{
    auto correspondences_3d2d
    (
        const std::map<size_t, point3d> & points,
        const std::map<size_t, keypoint> &keypoints,
        std::vector<cv::Point3d> &        points3d,
        std::vector<cv::Point2d> &        points2d,
        std::vector<size_t> &             indices
    ) -> void;


    void correspondences_3d3d
    (
        const std::map<size_t, point3d> &points_map_0,
        const std::map<size_t, point3d> &points_map_1,
        std::vector<cv::Point3d> &       points3d_0,
        std::vector<cv::Point3d> &       points3d_1,
        std::vector<size_t> &            indexes
    );

    auto estimate_pose_3d2d
    (
        const std::map<size_t, point3d> & map_points_0,
        const std::map<size_t, keypoint> &map_keypoints_1,
        const cv::Matx33d &               camera_matrix,
        const double &                    threshold
    ) -> pose_data;

    auto estimate_pose_3d3d
    (
        const std::map<size_t, point3d> &map_points_0,
        const std::map<size_t, point3d> &map_points_1,
        const double &                   threshold
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
        const map<keypoint> &map_keypoints_l,
        const map<keypoint> &map_keypoints_r,
        const cv::Matx33d &  fundamental,
        double               epipolar_threshold
    ) -> std::vector<cv::DMatch>;

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
    auto match_keylines
    (
        const map<keyline> &keylines_map_0,
        const map<keyline> &keylines_map_1,
        const cv::Matx33d & fundamental,
        double              epipolar_threshold
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
        const frame::camera &      frame,
        const camera_calibration & calibration,
        const class options::slam &options,
        const cv::Ptr<cv::CLAHE> & clahe
    ) -> frame::camera;

    /** Pre-process a camera frame by converting to grayscale, applying CLAHE, and building an image pyramid.
     *
     * @param frame The input stereo frame containing the image to be processed.
     * @param calibration The camera calibration parameters.
     * @param options SLAM options that may include CLAHE settings and pyramid levels.
     * @return The pre-processed frame.
     */
    auto pre_process
    (
        const frame::stereo &                    frame,
        const std::array<camera_calibration, 2> &calibration,
        const class options::slam &              options,
        const cv::Ptr<cv::CLAHE> &               clahe
    ) -> frame::stereo;

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
        const std::vector<cv::Mat> &    pyramid_0,
        const std::vector<cv::Mat> &    pyramid_1,
        const map<keypoint> &           keypoints_map_0,
        const class options::slam &     options,
        const std::vector<cv::Point2f> &points_1_predicted = { }
    ) -> std::vector<keypoint>;

    /** Track keypoints between two stereo frames.
     *
     * @param frames An array containing the two stereo frames to track between.
     * @param options SLAM options that may include KLT parameters.
     * @return An array of stereo frames with tracked keypoints.
     */
    auto track
    (
        const std::array<frame::stereo, 2> &frames,
        const class options::slam &         options
    ) -> std::array<std::vector<keypoint>, 2>;

    /** Track keylines from frame_0 to frame_1 using KLT optical flow on endpoints.
     *
     * This function tracks keylines by tracking their two endpoints independently using KLT.
     * Forward-backward tracking is used to filter out bad tracks. A keyline is considered
     * successfully tracked only if both endpoints are successfully tracked and pass the
     * forward-backward error check.
     *
     * @param pyramid_0 The image pyramid of the first frame.
     * @param pyramid_1 The image pyramid of the second frame.
     * @param keylines_map_0 A map of keylines in the first frame to be tracked.
     * @param options SLAM options that include KLT parameters (window size, max level, threshold).
     * @return A vector of tracked keylines in frame_1.
     */
    auto track_keylines
    (
        const std::vector<cv::Mat> &pyramid_0,
        const std::vector<cv::Mat> &pyramid_1,
        const map<keyline> &        keylines_map_0,
        const class options::slam & options
    ) -> std::vector<keyline>;

    /** Triangulate 3D points from stereo frame keypoints.
     *
     * This function triangulates 3D points from matched keypoints in a stereo frame.
     * It uses the provided projection matrices for the left and right cameras to perform
     * triangulation. Points with reprojection error below the specified threshold are returned.
     *
     * @param frame The stereo frame containing matched keypoints.
     * @param projection_0 The 3x4 projection matrix for the left camera.
     * @param projection_1 The 3x4 projection matrix for the right camera.
     * @param threshold The maximum allowable reprojection error for triangulated points.
     * @return A tuple containing a map of triangulated 3D points and a vector of reprojection errors.
     */
    auto triangulate
    (
        const frame::stereo &    frame,
        const cv::Matx34d &projection_0,
        const cv::Matx34d &projection_1,
        double             threshold
    ) -> map<point3d>;

    /**
     * Triangulate keylines between stereo frames using their indices.
     * For each keyline index present in both maps, triangulate endpoints and midpoint.
     *
     * @param keylines_l Map of keylines in left image
     * @param keylines_r Map of keylines in right image
     * @param P_l 3x4 projection matrix for left camera
     * @param P_r 3x4 projection matrix for right camera
     * @return Map from keyline index to tuple of 3D endpoints and midpoint
     */
    auto triangulate_keylines
    (
        const std::map<size_t, keyline> &keylines_l,
        const std::map<size_t, keyline> &keylines_r,
        const cv::Matx34d &              P_l,
        const cv::Matx34d &              P_r
    ) -> std::vector<line3d>;

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
    auto triangulate_points
    (
        const std::vector<cv::Point2f> &points2f_0,
        const std::vector<cv::Point2f> &points2f_1,
        const cv::Matx34d &             projection_0,
        const cv::Matx34d &             projection_1
    ) -> std::vector<cv::Point3d>;

    auto umeyama
    (
        const std::vector<cv::Point3d> &src,
        const std::vector<cv::Point3d> &dst,
        cv::Matx33d &                   R,
        cv::Point3d &                   t
    ) -> void;

    auto undistort(const cv::Mat &image, const camera_calibration &calibration) -> cv::Mat;
}
