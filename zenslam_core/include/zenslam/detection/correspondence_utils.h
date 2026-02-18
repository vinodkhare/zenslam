#pragma once

#include <map>
#include <vector>

#include <opencv2/core/types.hpp>

#include "zenslam/types/keyline.h"
#include "zenslam/types/keypoint.h"
#include "zenslam/types/line3d.h"
#include "zenslam/types/point3d.h"

namespace zenslam::utils
{
    /**
     * Returns 2D-2D correspondences between two sets of keypoints based on matching indices.
     * @param keypoints_0 0th set of keypoints
     * @param keypoints_1 1st set of keypoints
     * @param points2f_0 set of matching 2D points from 0th set
     * @param points2f_1 set of matching 2D points from 1st set
     * @param indices indices of 1 found in 0
     */
    auto correspondence_2d2d(
        const std::map<size_t, keypoint>& keypoints_0,
        const std::map<size_t, keypoint>& keypoints_1,
        std::vector<cv::Point2f>&         points2f_0,
        std::vector<cv::Point2f>&         points2f_1,
        std::vector<size_t>&              indices) -> void;

    auto correspondences_3d2d(
        const std::map<size_t, point3d>&  points,
        const std::map<size_t, keypoint>& keypoints,
        std::vector<cv::Point3d>&         points3d,
        std::vector<cv::Point2d>&         points2d,
        std::vector<size_t>&              indices) -> void;

    /**
     * Returns 3D-3D correspondences between two sets of 3D points based on matching indices.
     * @param points_map_0 0th set of 3D points
     * @param points_map_1 1st set of 3D points
     * @param points3d_0 set of matching 3D points from 0th set
     * @param points3d_1 set of matching 3D points from 1st set
     * @param indexes indices of 1 found in 0
     */
    void correspondences_3d3d(
        const std::map<size_t, point3d>& points_map_0,
        const std::map<size_t, point3d>& points_map_1,
        std::vector<cv::Point3d>&        points3d_0,
        std::vector<cv::Point3d>&        points3d_1,
        std::vector<size_t>&             indexes);

    /**
     * Returns 3D-2D line correspondences by matching 3D lines to 2D keylines based on indices.
     * For each matched index, extracts the two endpoints of the 3D line and 2D keyline.
     * This produces four separate vectors suitable for line-to-point pose estimation.
     *
     * @param lines_map Map of 3D lines (each containing two endpoints)
     * @param keylines_map Map of 2D keylines (each containing two endpoints)
     * @param lines3d_p1 First endpoints of matched 3D lines
     * @param lines3d_p2 Second endpoints of matched 3D lines
     * @param keylines2d_p1 First endpoints of matched 2D keylines
     * @param keylines2d_p2 Second endpoints of matched 2D keylines
     * @param indices Indices of matched correspondences
     */
    void correspondences_3d2d_lines(
        const std::map<size_t, line3d>&  lines_map,
        const std::map<size_t, keyline>& keylines_map,
        std::vector<cv::Point3d>&        lines3d_p1,
        std::vector<cv::Point3d>&        lines3d_p2,
        std::vector<cv::Point2d>&        keylines2d_p1,
        std::vector<cv::Point2d>&        keylines2d_p2,
        std::vector<size_t>&             indices);

    /**
     * Returns 3D-3D line correspondences by matching 3D lines between two frames.
     * For each matched index, extracts the two endpoints of each line.
     * This produces four separate 3D point vectors for line endpoint-based pose estimation.
     *
     * @param lines_map_0 Map of 3D lines from first frame
     * @param lines_map_1 Map of 3D lines from second frame
     * @param lines3d_0_p1 First endpoints from frame 0
     * @param lines3d_0_p2 Second endpoints from frame 0
     * @param lines3d_1_p1 First endpoints from frame 1
     * @param lines3d_1_p2 Second endpoints from frame 1
     * @param indices Indices of matched correspondences
     */
    void correspondences_3d3d_lines(
        const std::map<size_t, line3d>& lines_map_0,
        const std::map<size_t, line3d>& lines_map_1,
        std::vector<cv::Point3d>&       lines3d_0_p1,
        std::vector<cv::Point3d>&       lines3d_0_p2,
        std::vector<cv::Point3d>&       lines3d_1_p1,
        std::vector<cv::Point3d>&       lines3d_1_p2,
        std::vector<size_t>&            indices);
}
