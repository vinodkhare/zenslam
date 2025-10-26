#pragma once

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include "zenslam/calibration.h"
#include "zenslam/detector.h"
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

        /** Track keypoints from frame_0 to frame_1 and augment with new
         *  stereo keypoints and triangulated 3D points.
         *
         *  @param frame_0 Previous tracked frame.
         *  @param frame_1 Current processed frame.
         *  @return New tracked frame with keypoints tracked from frame_0 to
         *          frame_1, new stereo keypoints detected in frame_1, and
         *          triangulated 3D points.
         */
        [[nodiscard]] auto track(const frame::tracked& frame_0, const frame::processed& frame_1) const -> frame::tracked;

    private:
        calibration                    _calibration = { };
        class options::slam            _options     = { };
        cv::Ptr<cv::DescriptorMatcher> _matcher     = cv::DescriptorMatcher::create("BruteForce");
        detector                       _detector    = { };
    };
}
