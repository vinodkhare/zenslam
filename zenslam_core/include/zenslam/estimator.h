#pragma once

#include <map>

#include "zenslam/calibration.h"
#include "zenslam/frame/estimated.h"
#include "zenslam/frame/tracked.h"
#include "zenslam/options.h"
#include "zenslam/pose_data.h"
#include "zenslam/types/keyline.h"
#include "zenslam/types/line3d.h"
#include "zenslam/types/point3d.h"

namespace zenslam
{
    struct estimate_pose_result
    {
        pose_data    pose_3d3d;       // 3D-3D point result (maybe empty)
        pose_data    pose_3d2d;       // 3D-2D point result (maybe empty)
        pose_data    pose_2d2d;       // 2D-2D point result scaled using previous triangulated points (maybe empty)
        pose_data    pose_3d3d_lines; // 3D-3D line result (maybe empty)
        pose_data    pose_3d2d_lines; // 3D-2D line result (maybe empty)
        cv::Affine3d chosen_pose;     // Selected pose (or identity)
        size_t       chosen_count { };
    };

    class estimator
    {
    public:
        estimator(calibration calib, slam_options opts);

        /** Estimate the pose of frame 1 relative to frame 0 using 3D-2D and 3D-3D correspondences.
         *
         * @param points3d_0 Map of 3D points in frame 0
         * @param tracked_1 Tracked frame 1 containing keypoints and/or 3D points
         * @return Struct containing both pose estimates and the chosen pose
         */
        [[nodiscard]] auto estimate_pose(const std::map<size_t, point3d>& points3d_0, const frame::tracked& tracked_1) const -> estimate_pose_result;

        /** Estimate pose using 2D-2D essential matrix and scale from previous triangulated points, alongside 3D-2D and 3D-3D.
         *
         * @param frame_0 Previous estimated frame (provides previous keypoints and 3D points)
         * @param tracked_1 Current tracked frame
         */
        [[nodiscard]] auto estimate_pose(const frame::estimated& frame_0, const frame::tracked& tracked_1) const -> estimate_pose_result;

    private:
        [[nodiscard]] auto estimate_pose_3d2d(const std::map<size_t, point3d>& map_points_0, const std::map<size_t, keypoint>& map_keypoints_1) const
            -> pose_data;

        [[nodiscard]] auto estimate_pose_3d3d(const std::map<size_t, point3d>& map_points_0, const std::map<size_t, point3d>& map_points_1) const -> pose_data;

        [[nodiscard]] auto estimate_pose_2d2d(
            const std::map<size_t, keypoint>& map_keypoints_0,
            const std::map<size_t, keypoint>& map_keypoints_1,
            const std::map<size_t, point3d>&  map_points3d_0) const -> pose_data;

        [[nodiscard]] auto estimate_pose_3d2d_lines(const std::map<size_t, line3d>& map_lines_0, const std::map<size_t, keyline>& map_keylines_1) const
            -> pose_data;

        [[nodiscard]] auto estimate_pose_3d3d_lines(const std::map<size_t, line3d>& map_lines_0, const std::map<size_t, line3d>& map_lines_1) const -> pose_data;

    private:
        calibration  _calibration;
        slam_options _options;
    };
} // namespace zenslam
