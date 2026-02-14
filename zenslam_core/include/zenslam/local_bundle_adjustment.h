#pragma once

#include <cmath>
#include <cstddef>
#include <unordered_map>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>

#include "zenslam/frame/estimated.h"
#include "zenslam/slam_options.h"
#include "zenslam/types/point3d_cloud.h"

namespace zenslam
{
    class local_bundle_adjustment
    {
    public:
        struct result
        {
            bool   converged    = false;
            int    iterations   = 0;
            size_t residuals    = 0;
            double initial_rmse = std::nan("nan");
            double final_rmse   = std::nan("nan");
        };

        explicit local_bundle_adjustment(cv::Matx33d camera_matrix, lba_options options = {});

        auto optimize(std::unordered_map<size_t, frame::estimated>& keyframes, point3d_cloud& landmarks,
                      const std::vector<size_t>& fixed_keyframe_ids = {}) const -> result;

    private:
        cv::Matx33d _camera_matrix = {};
        lba_options _options       = {};
    };
} // namespace zenslam
