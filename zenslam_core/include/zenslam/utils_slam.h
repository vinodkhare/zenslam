#pragma once

#include <map>
#include <ranges>

#include <opencv2/features2d.hpp>

#include "zenslam/options.h"
#include "zenslam/types/keyline.h"
#include "zenslam/types/keypoint.h"
#include "zenslam/types/line3d.h"
#include "zenslam/types/point3d.h"
#include "zenslam/types/point3d_cloud.h"

inline auto operator-(const std::vector<cv::Point2d>& lhs, const std::vector<cv::Point2f>& rhs) -> std::vector<cv::Point2d>
{
    std::vector<cv::Point2d> difference;
    difference.reserve(lhs.size());

    for (auto i = 0; i < lhs.size(); ++i)
    {
        difference.emplace_back(lhs[i] - cv::Point2d(rhs[i].x, rhs[i].y));
    }

    return difference;
}


inline auto operator*(const cv::Affine3d& pose, const zenslam::line3d& line) -> zenslam::line3d
{
    zenslam::line3d transformed_line;

    transformed_line.index = line.index;
    transformed_line[0]    = pose * line[0];
    transformed_line[1]    = pose * line[1];

    return transformed_line;
}

inline auto operator*
(const cv::Affine3d& pose, const zenslam::map<zenslam::line3d>& lines) -> zenslam::map<zenslam::line3d>
{
    zenslam::map<zenslam::line3d> transformed_lines { };

    for (const auto line : lines | std::views::values)
    {
        transformed_lines[line.index]       = pose * line;
        transformed_lines[line.index].index = line.index;
    }

    return transformed_lines;
}

namespace zenslam::utils
{
    /**
     * Returns 2D-2D correspondences between two sets of keypoints based on matching indices.
     * @param keypoints_0 0th set of keypoints
     * @param keypoints_1 1st set of keypoints
     * @param points2f_0 set of matching 2D points from 0th set
     * @param points2f_1 set of matching 2D points from 1st set
     * @param indices indices of 1 found in 0
     */
    auto correspondence_2d2d
    (
        const std::map<size_t, keypoint>& keypoints_0,
        const std::map<size_t, keypoint>& keypoints_1,
        std::vector<cv::Point2f>&         points2f_0,
        std::vector<cv::Point2f>&         points2f_1,
        std::vector<size_t>&              indices
    ) -> void;

    auto correspondences_3d2d
    (
        const std::map<size_t, point3d>&  points,
        const std::map<size_t, keypoint>& keypoints,
        std::vector<cv::Point3d>&         points3d,
        std::vector<cv::Point2d>&         points2d,
        std::vector<size_t>&              indices
    ) -> void;


    /**
     * Returns 3D-3D correspondences between two sets of 3D points based on matching indices.
     * @param points_map_0 0th set of 3D points
     * @param points_map_1 1st set of 3D points
     * @param points3d_0 set of matching 3D points from 0th set
     * @param points3d_1 set of matching 3D points from 1st set
     * @param indexes indices of 1 found in 0
     */
    void correspondences_3d3d
    (
        const std::map<size_t, point3d>& points_map_0,
        const std::map<size_t, point3d>& points_map_1,
        std::vector<cv::Point3d>&        points3d_0,
        std::vector<cv::Point3d>&        points3d_1,
        std::vector<size_t>&             indexes
    );
    /**
     * Returns 3D-2D line correspondences by matching 3D lines to 2D keylines based on indices.
     * For each matched index, extracts the two endpoints of the 3D line and 2D keyline.
     * This produces four separate vectors suitable for line-to-point pose estimation.
     *
     * @param lines_map Map of 3D lines (each containing two endpoints)
     * @param keylines_map Map of 2D keylines (each containing two endpoints)
     * @param lines3d_p1 First endpoints of matched 3D lines
     * @param lines3d_p2 Second endpoints of matched 3D lines
     * @param keylines2d_p1 First endpoints of matched 2D keylines
     * @param keylines2d_p2 Second endpoints of matched 2D keylines
     * @param indices Indices of matched correspondences
     */
    void correspondences_3d2d_lines
    (
        const std::map<size_t, line3d>&  lines_map,
        const std::map<size_t, keyline>& keylines_map,
        std::vector<cv::Point3d>&        lines3d_p1,
        std::vector<cv::Point3d>&        lines3d_p2,
        std::vector<cv::Point2d>&        keylines2d_p1,
        std::vector<cv::Point2d>&        keylines2d_p2,
        std::vector<size_t>&             indices
    );

    /**
     * Returns 3D-3D line correspondences by matching 3D lines between two frames.
     * For each matched index, extracts the two endpoints of each line.
     * This produces four separate 3D point vectors for line endpoint-based pose estimation.
     *
     * @param lines_map_0 Map of 3D lines from first frame
     * @param lines_map_1 Map of 3D lines from second frame
     * @param lines3d_0_p1 First endpoints from frame 0
     * @param lines3d_0_p2 Second endpoints from frame 0
     * @param lines3d_1_p1 First endpoints from frame 1
     * @param lines3d_1_p2 Second endpoints from frame 1
     * @param indices Indices of matched correspondences
     */
    void correspondences_3d3d_lines
    (
        const std::map<size_t, line3d>& lines_map_0,
        const std::map<size_t, line3d>& lines_map_1,
        std::vector<cv::Point3d>&       lines3d_0_p1,
        std::vector<cv::Point3d>&       lines3d_0_p2,
        std::vector<cv::Point3d>&       lines3d_1_p1,
        std::vector<cv::Point3d>&       lines3d_1_p2,
        std::vector<size_t>&            indices
    );

    /** Estimate rigid transformation (R, t) between two sets of 3D points.
     *
     * @param points3d_0 First set of 3D points
     * @param points3d_1 Second set of 3D points
     * @param R Output rotation matrix
     * @param t Output translation vector
     * @return True if estimation was successful, false otherwise
     */
    auto estimate_rigid
    (
        const std::vector<cv::Point3d>& points3d_0,
        const std::vector<cv::Point3d>& points3d_1,
        cv::Matx33d&                    R,
        cv::Point3d&                    t
    ) -> bool;

    // RANSAC wrapper for estimate_rigid: returns best R, t, and inlier/outlier indices
    auto estimate_rigid_ransac
    (
        const std::vector<cv::Point3d>& src,
        const std::vector<cv::Point3d>& dst,
        cv::Matx33d&                    best_R,
        cv::Point3d&                    best_t,
        std::vector<size_t>&            inlier_indices,
        std::vector<size_t>&            outlier_indices,
        std::vector<double>&            errors,
        double                          threshold      = 0.01,
        int                             max_iterations = 1000
    ) -> bool;

    // filters matches using the epipolar criterion given the fundamental matrix
    auto filter
    (
        const std::vector<cv::KeyPoint>& keypoints0,
        const std::vector<cv::KeyPoint>& keypoints1,
        const std::vector<cv::DMatch>&   matches,
        const cv::Matx33d&               fundamental,
        double                           epipolar_threshold
    ) -> std::vector<cv::DMatch>;

    /**
     * Create a descriptor matcher based on options.
     * Creates the appropriate matcher (BFMatcher or FlannBasedMatcher) depending on
     * matcher type and descriptor type (binary vs float).
     *
     * @param options SLAM options containing matcher configuration
     * @param is_binary True for binary descriptors (ORB, BRISK), false for float (SIFT, SURF)
     * @return Pointer to the created matcher
     */
    auto create_matcher
    (
        const slam_options& options,
        bool                is_binary
    ) -> cv::Ptr<cv::DescriptorMatcher>;

    /**
     * Match 3D points to 2D keypoints by projecting the 3D points into the camera frame
     * and finding keypoints within a specified radius.
     *
     * @param points3d_world Map of 3D points in the world coordinate system
     * @param keypoints Map of 2D keypoints in the image
     * @param pose_of_camera0_in_world Affine transformation representing the camera pose in the world
     * @param projection
     * @param radius Radius (in pixels) for matching projected 3D points to 2D keypoints
     * @param threshold
     * @return Vector of cv::DMatch representing the matched 3D-2D correspondences
     */
    auto match_keypoints3d
    (
        const point3d_cloud& points3d_world,
        const map<keypoint>& keypoints,
        const cv::Affine3d&  pose_of_camera0_in_world,
        const cv::Matx34d&   projection,
        double               radius,
        double               threshold
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
        const map<keyline>& keylines_map_0,
        const map<keyline>& keylines_map_1,
        const cv::Matx33d&  fundamental,
        double              epipolar_threshold
    ) -> std::vector<cv::DMatch>;

    auto match_temporal
    (
        const std::map<size_t, keypoint>& keypoints_map_0,
        const std::map<size_t, keypoint>& keypoints_map_1,
        const cv::Matx33d&                camera_matrix,
        double                            threshold
    ) -> std::vector<cv::DMatch>;


    /** Solve the PnP problem to estimate camera pose from 3D-2D point correspondences.
     *
     * @param camera_matrix The intrinsic camera matrix.
     * @param points3d A vector of 3D points in the world coordinate system.
     * @param points2d A vector of corresponding 2D points in the image plane.
     * @param pose Output affine transformation representing the camera pose.
     */
    auto solve_pnp
    (
        const cv::Matx33d&              camera_matrix,
        const std::vector<cv::Point3d>& points3d,
        const std::vector<cv::Point2d>& points2d,
        cv::Affine3d&                   pose
    ) -> void;

    // Tracking moved to zenslam::tracker; keep lower-level helpers here.

    // track_keypoints moved into zenslam::tracker

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
        const std::vector<cv::Mat>& pyramid_0,
        const std::vector<cv::Mat>& pyramid_1,
        const map<keyline>&         keylines_map_0,
        const slam_options&         options
    ) -> std::vector<keyline>;

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
     * @param translation_of_camera1_in_camera0
     * @return Vector of 3D line segments with original keyline indices
     */
    auto triangulate_keylines
    (
        const map<keyline>& keylines_0,
        const map<keyline>& keylines_1,
        const cv::Matx34d&  projection_0,
        const cv::Matx34d&  projection_1,
        const slam_options& options,
        const cv::Vec3d&    translation_of_camera1_in_camera0
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
        const std::vector<cv::Point2f>& points2f_0,
        const std::vector<cv::Point2f>& points2f_1,
        const cv::Matx34d&              projection_0,
        const cv::Matx34d&              projection_1
    ) -> std::vector<cv::Point3d>;

    auto umeyama
    (
        const std::vector<cv::Point3d>& src,
        const std::vector<cv::Point3d>& dst,
        cv::Matx33d&                    R,
        cv::Point3d&                    t
    ) -> void;
}
