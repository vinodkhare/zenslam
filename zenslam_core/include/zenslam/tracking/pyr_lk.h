#pragma once

#include <memory>
#include <vector>

#include <opencv2/core.hpp>

namespace zenslam
{
    class pyr_lk
    {
    public:
        virtual ~pyr_lk() = default;

        virtual void calc_optical_flow_pyr_lk(
            const std::vector<cv::Mat>&     pyramid_0,
            const std::vector<cv::Mat>&     pyramid_1,
            const std::vector<cv::Point2f>& points_0,
            std::vector<cv::Point2f>&       points_1,
            std::vector<uchar>&             status,
            std::vector<float>&             errors,
            cv::Size                        window_size,
            int                             max_level,
            int                             flags
        ) const = 0;
    };

    [[nodiscard]] auto create_opencv_pyr_lk() -> std::shared_ptr<pyr_lk>;
} // namespace zenslam
