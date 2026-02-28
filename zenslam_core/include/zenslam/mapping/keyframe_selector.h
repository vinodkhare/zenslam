#pragma once

#include "zenslam/frame/estimated.h"
#include "zenslam/all_options.h"

namespace zenslam
{
    struct keyframe_decision
    {
        bool   is_keyframe       = false;
        size_t frames_since_last = 0;
        double translation       = 0.0;
        double rotation_deg      = 0.0;
        double tracked_ratio     = 1.0;
        size_t inliers           = 0;
        bool   forced            = false;
        bool   motion_triggered  = false;
        bool   quality_triggered = false;
    };

    class keyframe_selector
    {
    public:
        explicit keyframe_selector(const keyframe_options& options);

        [[nodiscard]] auto decide(const frame::estimated& frame) -> keyframe_decision;

        void reset();

    private:
        keyframe_options _options             = { };
        size_t           _last_keyframe_index = 0;
        cv::Affine3d     _last_keyframe_pose  = cv::Affine3d::Identity();
        bool             _has_keyframe        = false;

        static auto compute_rotation_deg(const cv::Affine3d& relative_pose) -> double;
        static auto compute_tracked_ratio(const frame::estimated& frame) -> double;
        static auto compute_inliers(const frame::estimated& frame) -> size_t;
    };
}
