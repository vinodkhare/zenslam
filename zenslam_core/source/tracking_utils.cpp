#include "zenslam/tracking_utils.h"

#include <cmath>
#include <ranges>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>

#include "zenslam/options.h"
#include "zenslam/types/keyline.h"

auto zenslam::utils::track_keylines(
    const std::vector<cv::Mat>& pyramid_0,
    const std::vector<cv::Mat>& pyramid_1,
    const map<keyline>&         keylines_map_0,
    const slam_options&         options) -> std::vector<keyline>
{
    if (keylines_map_0.empty())
    {
        return {};
    }

    // Extract start and end points of all keylines
    std::vector<cv::Point2f> start_points_0;
    std::vector<cv::Point2f> end_points_0;
    std::vector<keyline>     keylines_0;

    start_points_0.reserve(keylines_map_0.size());
    end_points_0.reserve(keylines_map_0.size());
    keylines_0.reserve(keylines_map_0.size());

    for (const auto& keyline : keylines_map_0 | std::views::values)
    {
        keylines_0.push_back(keyline);
        start_points_0.emplace_back(keyline.startPointX, keyline.startPointY);
        end_points_0.emplace_back(keyline.endPointX, keyline.endPointY);
    }

    // Forward tracking: track start and end points from frame 0 to frame 1
    auto               start_points_1 = start_points_0;
    auto               end_points_1   = end_points_0;
    std::vector<uchar> status_start_fwd;
    std::vector<uchar> status_end_fwd;
    std::vector<float> err_start_fwd;
    std::vector<float> err_end_fwd;

    cv::calcOpticalFlowPyrLK(
        pyramid_0,
        pyramid_1,
        start_points_0,
        start_points_1,
        status_start_fwd,
        err_start_fwd,
        options.klt_window_size,
        options.klt_max_level,
        cv::TermCriteria(
            cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
            99,
            0.001),
        cv::OPTFLOW_LK_GET_MIN_EIGENVALS);

    cv::calcOpticalFlowPyrLK(
        pyramid_0,
        pyramid_1,
        end_points_0,
        end_points_1,
        status_end_fwd,
        err_end_fwd,
        options.klt_window_size,
        options.klt_max_level,
        cv::TermCriteria(
            cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
            99,
            0.001),
        cv::OPTFLOW_LK_GET_MIN_EIGENVALS);

    // Backward tracking: track from frame 1 back to frame 0
    std::vector<cv::Point2f> start_points_0_back;
    std::vector<cv::Point2f> end_points_0_back;
    std::vector<uchar>       status_start_bwd;
    std::vector<uchar>       status_end_bwd;
    std::vector<float>       err_start_bwd;
    std::vector<float>       err_end_bwd;

    cv::calcOpticalFlowPyrLK(
        pyramid_1,
        pyramid_0,
        start_points_1,
        start_points_0_back,
        status_start_bwd,
        err_start_bwd,
        options.klt_window_size,
        options.klt_max_level,
        cv::TermCriteria(
            cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
            99,
            0.001),
        cv::OPTFLOW_LK_GET_MIN_EIGENVALS);

    cv::calcOpticalFlowPyrLK(
        pyramid_1,
        pyramid_0,
        end_points_1,
        end_points_0_back,
        status_end_bwd,
        err_end_bwd,
        options.klt_window_size,
        options.klt_max_level,
        cv::TermCriteria(
            cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
            99,
            0.001),
        cv::OPTFLOW_LK_GET_MIN_EIGENVALS);

    // Filter keylines based on forward-backward consistency
    std::vector<keyline> tracked_keylines;
    tracked_keylines.reserve(keylines_0.size());

    for (size_t i = 0; i < keylines_0.size(); ++i)
    {
        // Check if both endpoints were successfully tracked in both directions
        if (status_start_fwd[i] && status_end_fwd[i] && status_start_bwd[i] &&
            status_end_bwd[i])
        {
            // Compute forward-backward error for both endpoints
            const auto fb_error_start =
                static_cast<float>(cv::norm(start_points_0_back[i] - start_points_0[i]));
            const auto fb_error_end =
                static_cast<float>(cv::norm(end_points_0_back[i] - end_points_0[i]));

            // Accept the track only if both endpoints pass the forward-backward
            // threshold
            if (fb_error_start < options.klt_threshold && fb_error_end < options.klt_threshold)
            {
                auto tracked_keyline = keylines_0[i];

                // Update keyline endpoints with tracked positions
                tracked_keyline.startPointX = start_points_1[i].x;
                tracked_keyline.startPointY = start_points_1[i].y;
                tracked_keyline.endPointX   = end_points_1[i].x;
                tracked_keyline.endPointY   = end_points_1[i].y;

                // Update keyline midpoint (pt)
                tracked_keyline.pt.x = (start_points_1[i].x + end_points_1[i].x) * 0.5f;
                tracked_keyline.pt.y = (start_points_1[i].y + end_points_1[i].y) * 0.5f;

                // Update keyline length
                const auto dx = tracked_keyline.endPointX - tracked_keyline.startPointX;
                const auto dy = tracked_keyline.endPointY - tracked_keyline.startPointY;
                tracked_keyline.lineLength = std::sqrt(dx * dx + dy * dy);

                // Update keyline angle
                tracked_keyline.angle = std::atan2(dy, dx) * 180.0f / static_cast<float>(CV_PI);

                tracked_keylines.push_back(tracked_keyline);
            }
        }
    }

    return tracked_keylines;
}
