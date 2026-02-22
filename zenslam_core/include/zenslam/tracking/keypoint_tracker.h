#pragma once

#include <ranges>

#include <opencv2/core.hpp>

#include "zenslam/all_options.h"
#include "zenslam/calibration/calibration.h"
#include "zenslam/detection/keyline_detector.h"
#include "zenslam/detection/keypoint_detector.h"
#include "zenslam/detection/keypoint_detector_parallel.h"
#include "zenslam/frame/processed.h"
#include "zenslam/frame/system.h"
#include "zenslam/frame/tracked.h"
#include "zenslam/mapping/triangulator.h"
#include "zenslam/matching/matcher.h"
#include "zenslam/types/keypoint.h"
#include "zenslam/types/point3d_cloud.h"

namespace zenslam
{
    /** SLAM keypoint tracker: tracks keypoints between frames, detects new keypoints,
     *  and triangulates 3D points.
     */
    class keypoint_tracker
    {
    public:
        keypoint_tracker(calibration calib, slam_options opts, frame::system& system);

        /**
         * @brief Returns the tracked keypoints in the current frame corresponding to the input keypoints from the previous frame, with updated positions and tracking status.
         * @param frame_0 Frame containing keypoints to track from (previous frame).
         * @param frame_1 Frame to track keypoints to (current frame).
         * @return Tracked keypoints in the current frame corresponding to the input keypoints from the previous frame, with updated positions and tracking status.
         */
        [[nodiscard]] auto track(const frame::tracked& frame_0, const frame::processed& frame_1) const -> std::array<map<keypoint>, 2>;

        /**
         * @brief Triangulates 3D points from the tracked keypoints in the current frame.
         * @param keypoints Tracked keypoints in the current frame for both cameras.
         * @param image Optional color image to extract colors for the triangulated points.
         * @return Triangulated 3D points as a point cloud.
         */
        [[nodiscard]] auto triangulate(const std::array<map<keypoint>, 2>& keypoints, const cv::Mat& image) const -> point3d_cloud;

    private:
        calibration                _calibration                = { };
        tracking_options           _tracking                   = { };
        detection_options          _detection                  = { };
        triangulation_options      _triangulation              = { };
        matcher                    _matcher                    = { slam_options { }, false };
        triangulator               _triangulator               = { _calibration, slam_options { } };
        keypoint_detector          _keypoint_detector          = { _detection };
        keypoint_detector_parallel _keypoint_detector_parallel = { _detection };

        const frame::system& _system;

        /**
         *  @brief Assign landmark indices to keypoints based on proximity and descriptor match.
         *  @param keypoints Keypoints to assign landmark indices to.
         *  @param points3d  3D points to match against.
         *  @param camera_center Camera position in world coordinates.
         *  @param match_radius Radius around the camera to limit landmark candidates.
         *  @param max_descriptor_distance Maximum allowed descriptor distance for matching.
         */
        static auto assign_landmark_indices
        (
            std::vector<keypoint>& keypoints,
            const point3d_cloud&   points3d,
            const cv::Point3d&     camera_center,
            double                 match_radius,
            double                 max_descriptor_distance
        ) -> void;

        /**
         * @brief Returns the keypoints in the current frame corresponding to the input keypoints from the previous frame, with updated positions and tracking status.
         * @param pyramid_0 Image pyramid of the previous frame.
         * @param pyramid_1 Image pyramid of the current frame.
         * @param keypoints_map_0 Keypoints from the previous frame to track.
         * @return Keypoints in the current frame corresponding to the input keypoints, with updated positions and tracking status.
         */
        [[nodiscard]] auto track_keypoints(const std::vector<cv::Mat>& pyramid_0, const std::vector<cv::Mat>& pyramid_1, const map<keypoint>& keypoints_map_0) const -> std::vector<keypoint>;
    };
} // namespace zenslam
