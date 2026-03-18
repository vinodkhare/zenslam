#include "zenslam/tracking/pyr_lk.h"

#include <memory>

#include <opencv2/video/tracking.hpp>

namespace
{
    class opencv_pyr_lk final : public zenslam::pyr_lk
    {
    public:
        void calc_optical_flow_pyr_lk(
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
            const double min_eig_threshold) const override
        {
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
    };
}

auto zenslam::create_opencv_pyr_lk() -> std::shared_ptr<pyr_lk>
{
    return std::make_shared<opencv_pyr_lk>();
}
