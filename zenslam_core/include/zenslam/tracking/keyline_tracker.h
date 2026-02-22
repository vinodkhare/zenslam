#pragma once

#include <opencv2/core.hpp>

#include "zenslam/all_options.h"
#include "zenslam/calibration/calibration.h"
#include "zenslam/detection/detector.h"
#include "zenslam/frame/processed.h"
#include "zenslam/frame/system.h"
#include "zenslam/types/keyline.h"
#include "zenslam/types/line3d_cloud.h"

namespace zenslam
{
    /** SLAM keyline tracker: tracks keylines between frames, detects new keylines,
     *  and triangulates 3D lines.
     */
    class keyline_tracker
    {
    public:
        keyline_tracker(calibration calib, slam_options opts, frame::system& system);

        [[nodiscard]] auto track_keylines(
            const std::vector<cv::Mat>& pyramid_0,
            const std::vector<cv::Mat>& pyramid_1,
            const map<keyline>&         keylines_map_0) const -> std::vector<keyline>;

    private:
        calibration           _calibration        = {};
        tracking_options      _tracking           = {};
        detection_options     _detection          = {};
        triangulation_options _triangulation      = {};
        detector              _detector           = {};

        const frame::system& _system;

        /**
         *  @brief Assign landmark indices to keylines based on descriptor match.
         *  @param keylines Keylines to assign landmark indices to.
         *  @param lines3d  3D lines to match against.
         *  @param max_descriptor_distance Maximum allowed descriptor distance for matching.
         */
        static auto assign_landmark_indices(
            std::vector<keyline>& keylines,
            const line3d_cloud&   lines3d,
            double                max_descriptor_distance) -> void;

        friend class tracker;
    };
} // namespace zenslam
