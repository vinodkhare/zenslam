#pragma once

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include "zenslam/calibration.h"
#include "zenslam/detector.h"
#include "zenslam/matcher.h"
#include "zenslam/options.h"
#include "zenslam/frame/processed.h"
#include "zenslam/frame/tracked.h"

namespace zenslam
{
    /** SLAM tracker: tracks keypoints between frames, detects new keypoints,
     *  and triangulates 3D points.
     */
    class tracker
    {
    public:
        tracker(calibration calib, class options::slam opts);

        /** Track keypoints from frame_0 to frame_1 and augment with new stereo keypoints and triangulated 3D points.
         *
         *  @param frame_0 Previous tracked frame.
         *  @param frame_1 Current processed frame.
         *  @return New tracked frame with keypoints tracked from frame_0 to frame_1, new stereo keypoints detected in frame_1, and triangulated 3D points.
         */
        [[nodiscard]] auto track(const frame::tracked& frame_0, const frame::processed& frame_1) const -> frame::tracked;

    private:
        calibration                    _calibration = { };
        class options::slam            _options     = { };
        matcher                        _matcher = { _options, false };
        detector                       _detector    = { };

        /** Track keypoints from pyramid_0 to pyramid_1 using KLT optical flow.
         *
         *  @param pyramid_0 Image pyramid of the previous frame.
         *  @param pyramid_1 Image pyramid of the current frame.
         *  @param keypoints_map_0 Keypoints in the previous frame to track.
         *  @param points_1_predicted (Optional) Predicted locations of the keypoints in the current frame.
         *  @return Tracked keypoints in the current frame.
         */
        [[nodiscard]] auto track_keypoints(const std::vector<cv::Mat>& pyramid_0, const std::vector<cv::Mat>& pyramid_1, const map<keypoint>& keypoints_map_0, const std::vector<cv::Point2f>& points_1_predicted = { }) const -> std::vector<keypoint>;
    };
}
