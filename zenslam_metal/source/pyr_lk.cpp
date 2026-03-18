#include "zenslam_metal/pyr_lk.h"

#include <opencv2/video/tracking.hpp>

namespace zenslam::metal::detail
{
    auto metal_is_available() -> bool;

    void metal_calc_optical_flow_pyr_lk(
        const std::vector<cv::Mat>& prev_pyramid,
        const std::vector<cv::Mat>& next_pyramid,
        const std::vector<cv::Point2f>& prev_points,
        std::vector<cv::Point2f>& next_points,
        std::vector<uchar>& status,
        std::vector<float>& err,
        cv::Size win_size,
        int max_level,
        cv::TermCriteria criteria,
        int flags,
        double min_eig_threshold);
}

auto zenslam::metal::is_available() -> bool
{
#if defined(__APPLE__) && defined(ZENSLAM_METAL_ENABLED)
    return detail::metal_is_available();
#else
    return false;
#endif
}

void zenslam::metal::calc_optical_flow_pyr_lk(
    const std::vector<cv::Mat>& prev_pyramid,
    const std::vector<cv::Mat>& next_pyramid,
    const std::vector<cv::Point2f>& prev_points,
    std::vector<cv::Point2f>& next_points,
    std::vector<uchar>& status,
    std::vector<float>& err,
    const cv::Size win_size,
    const int max_level,
    const cv::TermCriteria criteria,
    const int flags,
    const double min_eig_threshold)
{
#if defined(__APPLE__) && defined(ZENSLAM_METAL_ENABLED)
    if (detail::metal_is_available())
    {
        detail::metal_calc_optical_flow_pyr_lk(
            prev_pyramid,
            next_pyramid,
            prev_points,
            next_points,
            status,
            err,
            win_size,
            max_level,
            criteria,
            flags,
            min_eig_threshold);
        return;
    }
#endif

    cv::calcOpticalFlowPyrLK(
        prev_pyramid,
        next_pyramid,
        prev_points,
        next_points,
        status,
        err,
        win_size,
        max_level,
        criteria,
        flags,
        min_eig_threshold);
}
