#include "zenslam/tracking/tracker.h"

#include <iostream>
#include <thread>
#include <utility>

#include "zenslam/time_this.h"
#include "zenslam/calibration/calibration.h"
#include "zenslam/frame/processed.h"
#include "zenslam/frame/system.h"
#include "zenslam/frame/tracked.h"
#include "zenslam/mapping/triangulation_utils.h"
#include "zenslam/matching/matching_utils.h"
#include "zenslam/tracking/keyline_tracker.h"
#include "zenslam/tracking/keypoint_tracker.h"
#include "zenslam/types/keyline.h"
#include "zenslam/types/keypoint.h"
#include "zenslam/types/map.h"

namespace zenslam
{
    tracker::tracker(calibration calib, slam_options opts, frame::system& system) :
        _keypoint_tracker(calib, opts, system),
        _keyline_tracker(calib, opts, system),
        _calibration(std::move(calib)),
        _epipolar_threshold(opts.epipolar_threshold),
        _system(system)
    {
    }

    tracker::~tracker() = default;

    auto tracker::track(const frame::tracked& frame_0, const frame::processed& frame_1) const -> frame::tracked
    {
        map<keyline>  keylines_0  = { };
        map<keyline>  keylines_1  = { };
        line3d_cloud  lines3d     = { };

        // Timing variables
        std::chrono::system_clock::duration time_stereo_tracking { 0 };
        std::chrono::system_clock::duration time_cross_tracking { 0 };
        std::chrono::system_clock::duration time_triangulation { 0 };
        std::chrono::system_clock::duration time_keyline_matching { 0 };
        std::chrono::system_clock::duration time_keyline_triangulation { 0 };
        std::chrono::system_clock::duration time_total { 0 };
        auto                                timer_total = time_this(time_total);

        std::vector<keypoint> keypoints_0_detected_all { };
        std::vector<keypoint> keypoints_1_detected_all { };

        std::array<map<keypoint>, 2> keypoints { };
        point3d_cloud                points3d { };
        {
            keypoints = _keypoint_tracker.track(frame_0, frame_1);
            points3d  = _keypoint_tracker.triangulate(keypoints, frame_1.undistorted[0]);
        }

        // Cross-camera (stereo) keyline tracking
        {
            map<keyline> keylines_0_untracked { };
            for (const auto& [index, keyline_0] : keylines_0) { if (!keylines_1.contains(index)) { keylines_0_untracked.emplace(index, keyline_0); } }

            auto keylines_1_tracked = _keyline_tracker.track_keylines(frame_1.pyramids[0], frame_1.pyramids[1], keylines_0_untracked);
            keylines_1.add(keylines_1_tracked);

            map<keyline> keylines_1_untracked { };
            for (const auto& [index, keyline_1] : keylines_1) { if (!keylines_0.contains(index)) { keylines_1_untracked.emplace(index, keyline_1); } }

            auto keylines_0_tracked = _keyline_tracker.track_keylines(frame_1.pyramids[1], frame_1.pyramids[0], keylines_1_untracked);
            keylines_0.add(keylines_0_tracked);
        }

        // Match keylines using the fundamental matrix
        {
            auto timer_kl_match = time_this(time_keyline_matching);

            const auto keyline_matches = utils::match_keylines(keylines_0, keylines_1, _calibration.fundamental_matrix[0], _epipolar_threshold);

            // Update keylines with matches (remap indices)
            keylines_1 *= keyline_matches;
        }

        // Triangulate keylines
        {
            slam_options options;
            options.triangulation = _keyline_tracker._triangulation;

            auto timer_kl_tri = time_this(time_keyline_triangulation);
            lines3d           += utils::triangulate_keylines
            (
                keylines_0,
                keylines_1,
                _calibration.projection_matrix[0],
                _calibration.projection_matrix[1],
                options,
                _calibration.cameras[1].pose_in_cam0.translation()
            );
        }

        // Log timing information
        auto ms = [](const auto& duration) { return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count(); };
        std::cout << "[TIMING] track() - Total: " << ms(time_total) << "ms "
            << "| Stereo: " << ms(time_stereo_tracking) << "ms "
            << "| Cross: " << ms(time_cross_tracking) << "ms "
            << "| Triangulation: " << ms(time_triangulation) << "ms "
            << "| KL Match: " << ms(time_keyline_matching) << "ms "
            << "| KL Tri: " << ms(time_keyline_triangulation) << "ms" << std::endl;

        return {
            frame_1,
            keypoints,
            { keylines_0, keylines_1 },
            points3d,
            lines3d
        };
    }
} // namespace zenslam
