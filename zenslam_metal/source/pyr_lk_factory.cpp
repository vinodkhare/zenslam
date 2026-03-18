#include "zenslam_metal/pyr_lk_factory.h"

#include <memory>

#include "zenslam_metal/pyr_lk.h"

namespace
{
    class metal_pyr_lk final : public zenslam::pyr_lk
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
            zenslam::metal::calc_optical_flow_pyr_lk(
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

auto zenslam::metal::create_metal_pyr_lk() -> std::shared_ptr<zenslam::pyr_lk>
{
    if (!is_available())
    {
        return {};
    }

    return std::make_shared<metal_pyr_lk>();
}
