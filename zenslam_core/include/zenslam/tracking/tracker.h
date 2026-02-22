#pragma once

#include <opencv2/core.hpp>

#include "zenslam/all_options.h"
#include "zenslam/calibration/calibration.h"
#include "zenslam/detection/detector.h"
#include "zenslam/frame/processed.h"
#include "zenslam/frame/system.h"
#include "zenslam/frame/tracked.h"
#include "zenslam/mapping/triangulator.h"
#include "zenslam/matching/matcher.h"
#include "zenslam/types/keyline.h"
#include "zenslam/types/line3d_cloud.h"

namespace zenslam
{
    /** SLAM tracker: tracks keypoints between frames, detects new keypoints,
     *  and triangulates 3D points.
     */
    class tracker
    {
    public:
        tracker(calibration calib, slam_options opts, frame::system& system);

        /** Track keypoints from frame_0 to frame_1 and augment with new stereo keypoints and triangulated 3D points.
         *
         *  @param frame_0 Previous tracked frame.
         *  @param frame_1 Current processed frame.
         *  @return New tracked frame with keypoints tracked from frame_0 to frame_1, new stereo keypoints detected in frame_1, and triangulated 3D points.
         */
        [[nodiscard]] auto track(const frame::tracked& frame_0, const frame::processed& frame_1) const -> frame::tracked;

    private:
        calibration           _calibration        = {};
        tracking_options      _tracking           = {};
        detection_options     _detection          = {};
        triangulation_options _triangulation      = {};
        double                _epipolar_threshold = 1.0;
        matcher               _matcher            = {slam_options{}, false};
        triangulator          _triangulator       = {_calibration, slam_options{}};
        detector              _detector           = {};

        const frame::system& _system;

        /** Track keypoints from pyramid_0 to pyramid_1 using KLT optical flow.
         *
         *  @param pyramid_0 Image pyramid of the previous frame.
         *  @param pyramid_1 Image pyramid of the current frame.
         *  @param keypoints_map_0 Keypoints in the previous frame to track.
         *  @return Tracked keypoints in the current frame.
         */
        [[nodiscard]] auto track_keypoints
        (
            const std::vector<cv::Mat>& pyramid_0,
            const std::vector<cv::Mat>& pyramid_1,
            const map<keypoint>&        keypoints_map_0
        ) const
            -> std::vector<keypoint>;

        /** Track keylines from pyramid_0 to pyramid_1 using KLT optical flow on endpoints.
         *
         *  @param pyramid_0 Image pyramid of the previous frame.
         *  @param pyramid_1 Image pyramid of the current frame.
         *  @param keylines_map_0 Keylines in the previous frame to track.
         *  @return Tracked keylines in the current frame.
         */
        [[nodiscard]] auto track_keylines(const std::vector<cv::Mat>& pyramid_0, const std::vector<cv::Mat>& pyramid_1, const map<keyline>& keylines_map_0) const -> std::vector<keyline>;

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
         *  @brief Assign landmark indices to keylines based on descriptor match.
         *  @param keylines Keylines to assign landmark indices to.
         *  @param lines3d  3D lines to match against.
         *  @param max_descriptor_distance Maximum allowed descriptor distance for matching.
         */
        static auto assign_landmark_indices
        (
            std::vector<keyline>& keylines,
            const line3d_cloud&   lines3d,
            double                max_descriptor_distance
        ) -> void;
    };
} // namespace zenslam
