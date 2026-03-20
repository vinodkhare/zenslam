#pragma once

#include <vector>

#include <opencv2/core.hpp>

namespace zenslam::metal
{
    /**
     * @brief Metal-accelerated Lucas-Kanade pyramid optical flow.
     *
     * Drop-in replacement for cv::calcOpticalFlowPyrLK that executes on the
     * Apple GPU via Metal compute shaders.
     *
     * @param pyramid_0     Image pyramid of the previous frame (CV_8UC1, each level half-sized).
     * @param pyramid_1     Image pyramid of the current frame (CV_8UC1).
     * @param points_0      Input 2D points in pyramid_0 to track.
     * @param points_1      [in/out] Initial guess on entry; tracked positions on exit.
     * @param status        Output per-point tracking status (1 = tracked, 0 = lost).
     * @param errors        Output per-point tracking errors.
     * @param window_size   KLT search window half-size (e.g. cv::Size(21,21)).
     * @param max_level     Maximum pyramid level (0 = image only).
     * @param flags         OPTFLOW_USE_INITIAL_FLOW and/or OPTFLOW_LK_GET_MIN_EIGENVALS.
     */
    void calc_optical_flow_pyr_lk(
        const std::vector<cv::Mat>&     pyramid_0,
        const std::vector<cv::Mat>&     pyramid_1,
        const std::vector<cv::Point2f>& points_0,
        std::vector<cv::Point2f>&       points_1,
        std::vector<uchar>&             status,
        std::vector<float>&             errors,
        cv::Size                        window_size = cv::Size(21, 21),
        int                             max_level   = 3,
        int                             flags       = 0
    );

} // namespace zenslam::metal
