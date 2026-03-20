#include "zenslam_metal/pyr_lk_factory.h"

#include "zenslam_metal/pyr_lk.h"

namespace zenslam::metal
{
    namespace
    {
        class metal_pyr_lk final : public zenslam::pyr_lk
        {
        public:
            void calc_optical_flow_pyr_lk(
                const std::vector<cv::Mat>&     pyramid_0,
                const std::vector<cv::Mat>&     pyramid_1,
                const std::vector<cv::Point2f>& points_0,
                std::vector<cv::Point2f>&       points_1,
                std::vector<uchar>&             status,
                std::vector<float>&             errors,
                const cv::Size                  window_size,
                const int                       max_level,
                const int                       flags
            ) const override
            {
                zenslam::metal::calc_optical_flow_pyr_lk(
                    pyramid_0,
                    pyramid_1,
                    points_0,
                    points_1,
                    status,
                    errors,
                    window_size,
                    max_level,
                    flags
                );
            }
        };
    } // namespace

    auto create_metal_pyr_lk() -> std::shared_ptr<zenslam::pyr_lk>
    {
        return std::make_shared<metal_pyr_lk>();
    }
} // namespace zenslam::metal
