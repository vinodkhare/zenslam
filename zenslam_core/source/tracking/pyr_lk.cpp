#include "zenslam/tracking/pyr_lk.h"

#include <opencv2/video/tracking.hpp>

namespace zenslam
{
    namespace
    {
        class opencv_pyr_lk final : public pyr_lk
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
                cv::calcOpticalFlowPyrLK(
                    pyramid_0,
                    pyramid_1,
                    points_0,
                    points_1,
                    status,
                    errors,
                    window_size,
                    max_level,
                    cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 99, 0.001),
                    flags
                );
            }
        };
    } // namespace

    auto create_opencv_pyr_lk() -> std::shared_ptr<pyr_lk>
    {
        return std::make_shared<opencv_pyr_lk>();
    }
} // namespace zenslam
