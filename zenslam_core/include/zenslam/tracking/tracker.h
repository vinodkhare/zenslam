#pragma once

#include <opencv2/core.hpp>

#include "zenslam/calibration/calibration.h"
#include "zenslam/detection/detector.h"
#include "zenslam/matching/matcher.h"
#include "zenslam/options.h"
#include "zenslam/mapping/triangulator.h"
#include "zenslam/frame/estimated.h"
#include "zenslam/frame/processed.h"
#include "zenslam/frame/tracked.h"
#include "zenslam/types/keyline.h"

namespace zenslam
{
    /** SLAM tracker: tracks keypoints between frames, detects new keypoints,
     *  and triangulates 3D points.
     */
    class tracker
    {
    public:
        tracker(calibration calib, slam_options opts);

        /** Track keypoints from frame_0 to frame_1 and augment with new stereo keypoints and triangulated 3D points.
         *
         *  @param frame_0 Previous tracked frame.
         *  @param frame_1 Current processed frame.
         *  @return New tracked frame with keypoints tracked from frame_0 to frame_1, new stereo keypoints detected in frame_1, and triangulated 3D points.
         */
        [[nodiscard]] auto track(const frame::tracked& frame_0, const frame::processed& frame_1) const -> frame::tracked;

        /** Track keypoints using a predicted pose to initialize KLT with better guesses.
         *
         *  The predicted absolute pose is used to project known 3D points from the previous frame
         *  into the current frame, providing initial estimates for optical flow. For keypoints
         *  without an associated 3D point, their previous 2D location is used as the initial guess.
         *
         *  @param frame_0 Previous estimated frame (contains pose and tracked data).
         *  @param frame_1 Current processed frame.
         *  @param pose_predicted Predicted absolute pose of the current frame (camera-0).
         *  @return New tracked frame.
         */
        [[nodiscard]] auto track(const frame::estimated& frame_0, const frame::processed& frame_1, const cv::Affine3d& pose_predicted) const -> frame::tracked;

    private:
        calibration  _calibration  = { };
        slam_options _options      = { };
        matcher      _matcher      = { _options, false };
        triangulator _triangulator = { _calibration, _options };
        detector     _detector     = { };

        /** Track keypoints from pyramid_0 to pyramid_1 using KLT optical flow.
         *
         *  @param pyramid_0 Image pyramid of the previous frame.
         *  @param pyramid_1 Image pyramid of the current frame.
         *  @param keypoints_map_0 Keypoints in the previous frame to track.
         *  @param points_1_predicted (Optional) Predicted locations of the keypoints in the current frame.
         *  @return Tracked keypoints in the current frame.
         */
        [[nodiscard]] auto track_keypoints
        (
            const std::vector<cv::Mat>&     pyramid_0,
            const std::vector<cv::Mat>&     pyramid_1,
            const map<keypoint>&            keypoints_map_0,
            const std::vector<cv::Point2f>& points_1_predicted = { }
        ) const -> std::vector<keypoint>;

        /** Track keylines from pyramid_0 to pyramid_1 using KLT optical flow on endpoints.
         *
         *  @param pyramid_0 Image pyramid of the previous frame.
         *  @param pyramid_1 Image pyramid of the current frame.
         *  @param keylines_map_0 Keylines in the previous frame to track.
         *  @return Tracked keylines in the current frame.
         */
        [[nodiscard]] auto track_keylines
        (
            const std::vector<cv::Mat>& pyramid_0,
            const std::vector<cv::Mat>& pyramid_1,
            const map<keyline>&         keylines_map_0
        ) const -> std::vector<keyline>;
    };
}
