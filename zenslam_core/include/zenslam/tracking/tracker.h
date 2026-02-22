#pragma once

#include "zenslam/all_options.h"
#include "zenslam/calibration/calibration.h"
#include "zenslam/frame/processed.h"
#include "zenslam/frame/system.h"
#include "zenslam/frame/tracked.h"
#include "zenslam/tracking/keyline_tracker.h"
#include "zenslam/tracking/keypoint_tracker.h"

namespace zenslam
{
    /** Unified SLAM tracker coordinator: manages separate keypoint and keyline trackers,
     *  performing both feature tracking and 3D point/line triangulation in parallel.
     */
    class tracker
    {
    public:
        tracker(calibration calib, slam_options opts, frame::system& system);

        ~tracker();

        /** Track keypoints and keylines from frame_0 to frame_1 and augment with new features and triangulated 3D data.
         *
         *  @param frame_0 Previous tracked frame.
         *  @param frame_1 Current processed frame.
         *  @return New tracked frame with tracked/detected keypoints and keylines, and triangulated 3D points and lines.
         */
        [[nodiscard]] auto track(const frame::tracked& frame_0, const frame::processed& frame_1) const -> frame::tracked;

    private:
        tracking_options _options = { };
        keypoint_tracker _keypoint_tracker;
        keyline_tracker  _keyline_tracker;
        calibration      _calibration;
        double           _epipolar_threshold = 1.0;

        const frame::system& _system;
    };
} // namespace zenslam
