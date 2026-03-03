#include "zenslam/mapping/keyframe_selector.h"

#include <algorithm>

#include <opencv2/calib3d.hpp>

namespace zenslam
{
    keyframe_selector::keyframe_selector(const keyframe_options& options)
        : _options(options)
    {
    }

    auto keyframe_selector::decide(const frame::estimated& frame) -> keyframe_decision
    {
        keyframe_decision decision { };

        if (!_has_keyframe)
        {
            _has_keyframe        = true;
            _last_keyframe_pose  = frame.pose;
            _last_keyframe_index = frame.index;

            decision.is_keyframe = true;
            return decision;
        }

        decision.frames_since_last = frame.index - _last_keyframe_index;

        if (decision.frames_since_last < static_cast<size_t>(_options.min_frames))
        {
            return decision;
        }

        const auto relative = _last_keyframe_pose.inv() * frame.pose;

        decision.translation   = cv::norm(relative.translation());
        decision.rotation_deg  = compute_rotation_deg(relative);
        decision.tracked_ratio = compute_tracked_ratio(frame);
        decision.inliers       = compute_inliers(frame);

        decision.forced            = decision.frames_since_last >= static_cast<size_t>(_options.max_frames);
        decision.motion_triggered  = decision.translation >= _options.min_translation || decision.rotation_deg >= _options.min_rotation_deg;
        decision.quality_triggered = decision.tracked_ratio <= _options.min_tracked_ratio || decision.inliers < static_cast<size_t>(_options.min_inliers);

        if (decision.forced || decision.motion_triggered || decision.quality_triggered)
        {
            decision.is_keyframe = true;

            _last_keyframe_pose  = frame.pose;
            _last_keyframe_index = frame.index;
        }

        return decision;
    }

    void keyframe_selector::reset()
    {
        _last_keyframe_index = 0;
        _last_keyframe_pose  = cv::Affine3d::Identity();
        _has_keyframe        = false;
    }

    auto keyframe_selector::compute_rotation_deg(const cv::Affine3d& relative_pose) -> double
    {
        cv::Vec3d rvec;
        cv::Rodrigues(relative_pose.rotation(), rvec);
        const auto angle_rad = cv::norm(rvec);
        return angle_rad * (180.0 / CV_PI);
    }

    auto keyframe_selector::compute_tracked_ratio(const frame::estimated& frame) -> double
    {
        const auto total_features =
            frame.keypoints[0].size() +
            frame.keypoints[1].size() +
            frame.keylines[0].size() +
            frame.keylines[1].size();

        if (total_features == 0)
        {
            return 1.0;
        }

        const auto stable_features = frame.points3d.size() + frame.lines3d.size();
        const auto ratio = static_cast<double>(stable_features) / static_cast<double>(total_features);
        return std::clamp(ratio, 0.0, 1.0);
    }

    auto keyframe_selector::compute_inliers(const frame::estimated& frame) -> size_t
    {
        return std::max(frame.points3d.size(), frame.lines3d.size());
    }
}
