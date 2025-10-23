#pragma once

#include <map>
#include <ranges>

#include <opencv2/imgproc.hpp>

#include "frame/slam.h"

#include "zenslam/calibration.h"
#include "zenslam/options.h"
#include "zenslam/pose_data.h"
#include "zenslam/frame/processed.h"
#include "zenslam/frame/sensor.h"
#include "zenslam/frame/stereo.h"
#include "zenslam/frame/tracked.h"
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

inline auto operator*(const cv::Affine3d& pose, const zenslam::map<zenslam::line3d>& lines) -> zenslam::map<zenslam::line3d>
{
    zenslam::map<zenslam::line3d> transformed_lines { };

    for (const auto line: lines | std::views::values)
    {
        transformed_lines[line.index]       = pose * line;
        transformed_lines[line.index].index = line.index;
    }

    return transformed_lines;
}

namespace zenslam::utils
{
    auto correspondences_3d2d
    (
        const std::map<size_t, point3d>&  points,
        const std::map<size_t, keypoint>& keypoints,
        std::vector<cv::Point3d>&         points3d,
        std::vector<cv::Point2d>&         points2d,
        std::vector<size_t>&              indices
    ) -> void;


    void correspondences_3d3d
    (
        const std::map<size_t, point3d>& points_map_0,
        const std::map<size_t, point3d>& points_map_1,
        std::vector<cv::Point3d>&        points3d_0,
        std::vector<cv::Point3d>&        points3d_1,
        std::vector<size_t>&             indexes
    );

    auto estimate_pose_3d2d
    (
        const std::map<size_t, point3d>&  map_points_0,
        const std::map<size_t, keypoint>& map_keypoints_1,
        const cv::Matx33d&                camera_matrix,
        const double&                     threshold
    ) -> pose_data;

    auto estimate_pose_3d3d
    (
        const std::map<size_t, point3d>& map_points_0,
        const std::map<size_t, point3d>& map_points_1,
        const double&                    threshold
    ) -> pose_data;

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

    // filters matches using the epipolar crterion given the fundamental matrix
    auto filter
    (
        const std::vector<cv::KeyPoint>& keypoints0,
        const std::vector<cv::KeyPoint>& keypoints1,
        const std::vector<cv::DMatch>&   matches,
        const cv::Matx33d&               fundamental,
        double                           epipolar_threshold
    ) -> std::vector<cv::DMatch>;

    auto match_keypoints
    (
        const map<keypoint>& keypoints_0,
        const map<keypoint>& keypoints_1,
        const cv::Matx33d&   fundamental,
        double               epipolar_threshold
    ) -> std::vector<cv::DMatch>;

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

    /** Pre-process a sensor frame by converting to grayscale, applying CLAHE, and building an image pyramid.
     *
     * @param sensor The input sensor frame containing the image to be processed.
     * @param calibration The camera calibration parameters.
     * @param options SLAM options that may include CLAHE settings and pyramid levels.
     * @return The pre-processed frame.
     */
    auto pre_process
    (
        const frame::sensor&       sensor,
        const calibration&         calibration,
        const class options::slam& options,
        const cv::Ptr<cv::CLAHE>&  clahe
    ) -> frame::processed;


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

    /** Track keypoints and keylines between frames.
     *
     * @param frame_0 The previous SLAM frame containing keypoints and keylines.
     * @param frame_1 The processed frame containing image pyramids.
     * @param calibration The camera calibration parameters.
     * @param options SLAM options that may include KLT parameters.
     * @return The tracked frame with updated keypoints and keylines.
     */
    auto track
    (const frame::tracked& frame_0, const frame::processed& frame_1, const zenslam::calibration& calibration, const class options::slam& options) -> frame::tracked;

    /** Track keypoints from frame_0 to frame_1 using KLT optical flow.
     *
     * @param pyramid_0 The image pyramid of the first frame.
     * @param pyramid_1 The image pyramid of the second frame.
     * @param keypoints_map_0 A map of keypoints in the first frame to be tracked.
     * @param options SLAM options that may include KLT parameters.
         * @param camera_matrix Camera intrinsic matrix for essential matrix filtering.
     * @param points_1_predicted Optional predicted positions of keypoints in frame_1 for improved tracking.
     * @return A vector of tracked keypoints in frame_1.
     */
    auto track_keypoints
    (
        const std::vector<cv::Mat>&     pyramid_0,
        const std::vector<cv::Mat>&     pyramid_1,
        const map<keypoint>&            keypoints_map_0,
        const class options::slam&      options,
        const cv::Matx33d&              camera_matrix,
        const std::vector<cv::Point2f>& points_1_predicted = { }
    ) -> std::vector<keypoint>;

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
        const class options::slam&  options
    ) -> std::vector<keyline>;

    /** Triangulate keypoints between stereo frames using their indices.
     * For each keypoint index present in both maps, triangulate the 3D point.
     *
     * @param keypoints_0 Map of keypoints in left image
     * @param keypoints_1 Map of keypoints in right image
     * @param projection_0 3x4 projection matrix for left camera
     * @param projection_1 3x4 projection matrix for right camera
     * @param triangulation_threshold Maximum allowable reprojection error
     * @param translation_of_camera1_in_camera0
     * @return Map from keypoint index to triangulated 3D point
     */
    auto triangulate_keypoints
    (
        const map<keypoint>& keypoints_0,
        const map<keypoint>& keypoints_1,
        const cv::Matx34d&   projection_0,
        const cv::Matx34d&   projection_1,
        double               triangulation_threshold,
        const cv::Vec3d&     translation_of_camera1_in_camera0
    ) -> std::vector<point3d>;

    /**
     * Triangulate keylines between stereo frames using their indices.
     * For each keyline index present in both maps, triangulate the two endpoints;
     * the returned line3d stores just the 3D endpoints (no separate midpoint).
     *
     * @param keylines_0 Map of keylines in left image
     * @param keylines_1 Map of keylines in right image
     * @param projection_0 3x4 projection matrix for left camera
     * @param projection_1 3x4 projection matrix for right camera
     * @param translation_of_camera1_in_camera0
     * @return Vector of 3D line segments with original keyline indices
     */
    auto triangulate_keylines
    (
        const map<keyline>&        keylines_0,
        const map<keyline>&        keylines_1,
        const cv::Matx34d&         projection_0,
        const cv::Matx34d&         projection_1,
        const class options::slam& options,
        const cv::Vec3d&           translation_of_camera1_in_camera0
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
