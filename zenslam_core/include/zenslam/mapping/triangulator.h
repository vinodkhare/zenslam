#pragma once

#include <vector>

#include "zenslam/calibration/calibration.h"
#include "zenslam/all_options.h"
#include "zenslam/types/keyline.h"
#include "zenslam/types/keypoint.h"
#include "zenslam/types/line3d_cloud.h"
#include "zenslam/types/map.h"
#include "zenslam/types/point3d_cloud.h"

namespace zenslam
{
    /** Triangulator: triangulates 3D points from stereo keypoint correspondences.
     */
    class triangulator
    {
    public:
        triangulator(calibration calib, const slam_options& opts);

        /**
         * Triangulate keypoints between stereo frames using their indices.
         * For each keypoint index present in both maps, triangulate its 3D position;
         * filters by reprojection error, parallax angle, and positive depth.
         *
         * @param keypoints_0 Map of keypoints in left image
         * @param keypoints_1 Map of keypoints in right image
         * @param color_image Optional color image (BGR) to sample colors from keypoints_0
         * @return Vector of triangulated 3D points that pass quality thresholds
         */
        [[nodiscard]] auto triangulate_keypoints(const map<keypoint>& keypoints_0, const map<keypoint>& keypoints_1, const cv::Mat& color_image = cv::Mat()) const -> point3d_cloud;

        /**
         * Triangulate keylines between stereo frames using their indices.
         * For each keyline index present in both maps, triangulate the two endpoints;
         * the returned @ref zenslam::line3d stores just the 3D endpoints (no separate midpoint).
         *
         * @param keylines_0 Map of keylines in left image
         * @param keylines_1 Map of keylines in right image
         * @param color_image Optional color image (BGR) to sample colors from keylines_0
         * @return Vector of triangulated 3D line segments with original keyline indices
         */
        [[nodiscard]] auto triangulate_keylines(const map<keyline>& keylines_0, const map<keyline>& keylines_1, const cv::Mat& color_image = cv::Mat()) const -> line3d_cloud;

    private:
        calibration  _calibration { };
        slam_options _options { };

        [[nodiscard]] auto filter_epipolar(const map<keypoint>& keypoints_0, const map<keypoint>& keypoints_1) const -> std::vector<std::pair<keypoint, keypoint>>;
    };
}
