#pragma once

#include <opencv2/core.hpp>

#include "zenslam/all_options.h"
#include "zenslam/calibration/calibration.h"
#include "zenslam/detection/keyline_detector.h"
#include "zenslam/frame/processed.h"
#include "zenslam/frame/system.h"
#include "zenslam/mapping/triangulator.h"
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

        /**
         * @brief Returns the tracked keylines in the current frame corresponding to the input keylines from the previous frame, with updated positions and tracking status, along with new detections and triangulated 3D lines added.
         * @param frame_0 Frame containing keylines to track from (previous frame).
         * @param frame_1 Frame to track keylines to (current frame).
         * @return keylines tracked from frame_0 to frame_1, with new detections and triangulated 3D lines added
         */
        [[nodiscard]] auto track(const frame::tracked& frame_0, const frame::processed& frame_1) const -> std::array<map<keyline>, 2>;

        /**
         * @brief Triangulates 3D lines from the tracked keylines in the current frame.
         * @param keylines Tracked keylines in the current frame for both cameras.
         * @param image Optional color image to extract colors for the triangulated lines.
         * @return Triangulated 3D lines as a line cloud.
         */
        [[nodiscard]] auto triangulate(const std::array<map<keyline>, 2>& keylines, const cv::Mat& image) const -> line3d_cloud;

    private:
        detection_options     _options       = { };
        calibration           _calibration   = { };
        tracking_options      _tracking      = { };
        triangulation_options _triangulation = { };
        keyline_detector      _detector      = { _options };
        triangulator          _triangulator;

        const frame::system& _system;

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

        /**
         *  @brief Track keylines from pyramid_0 to pyramid_1 using optical flow, given the keylines in pyramid_0.
         *  @param pyramid_0 Image pyramid for the previous frame.
         *  @param pyramid_1 Image pyramid for the current frame.
         *  @param keylines_map_0 Map of keylines in the previous frame to track from.
         *  @return Vector of tracked keylines in the current frame corresponding to the input keylines from the previous frame, with updated positions and tracking status.
         */
        [[nodiscard]] auto track_keylines(const std::vector<cv::Mat>& pyramid_0, const std::vector<cv::Mat>& pyramid_1, const map<keyline>& keylines_map_0) const -> std::vector<keyline>;
    };
} // namespace zenslam
