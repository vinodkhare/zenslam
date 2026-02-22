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
        _options { opts.tracking },
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
        // Timing variables
        std::chrono::system_clock::duration time_keypoint_tracking { 0 };
        std::chrono::system_clock::duration time_keypoint_triangulation { 0 };
        std::chrono::system_clock::duration time_keyline_tracking { 0 };
        std::chrono::system_clock::duration time_keyline_triangulation { 0 };
        std::chrono::system_clock::duration time_total { 0 };

        std::array<map<keypoint>, 2> keypoints { };
        point3d_cloud                points3d { };
        std::array<map<keyline>, 2>  keylines = { };
        line3d_cloud                 lines3d  = { };
        {
            time_this time_this_total { time_total };

            {
                time_this time_this { time_keypoint_tracking };
                keypoints = _keypoint_tracker.track(frame_0, frame_1);
            }

            {
                time_this time_this { time_keypoint_triangulation };
                points3d = _keypoint_tracker.triangulate(keypoints, frame_1.undistorted[0]);
            }

            if (_options.use_keylines)
            {
                time_this time_this { time_keyline_tracking };
                keylines = _keyline_tracker.track(frame_0, frame_1);
            }

            if (_options.use_keylines)
            {
                time_this time_this { time_keyline_triangulation };
                lines3d = _keyline_tracker.triangulate(keylines, frame_1.undistorted[0]);
            }
        }

        // Log timing information
        auto ms = [](const auto& duration) { return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count(); };
        std::cout << "[TIMING] track() - Total: " << ms(time_total) << "ms "
            << "| Stereo: " << ms(time_keypoint_tracking) << "ms "
            << "| Triangulation: " << ms(time_keypoint_triangulation) << "ms "
            << "| KL Match: " << ms(time_keyline_tracking) << "ms "
            << "| KL Tri: " << ms(time_keyline_triangulation) << "ms" << std::endl;

        return { frame_1, keypoints, keylines, points3d, lines3d };
    }
} // namespace zenslam
