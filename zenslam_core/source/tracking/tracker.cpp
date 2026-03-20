#include "zenslam/tracking/tracker.h"

#include <future>
#include <utility>

#include <spdlog/spdlog.h>

#include "zenslam/time_this.h"

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

    auto tracker::track(const frame::estimated& frame_0, const frame::processed& frame_1, const cv::Affine3d& predicted_pose) const -> frame::tracked
    {
        std::chrono::system_clock::duration time_total { 0 };

        std::array<map<keypoint>, 2> keypoints { };
        point3d_cloud                points3d { };
        std::array<map<keyline>, 2>  keylines = { };
        line3d_cloud                 lines3d  = { };
        {
            time_this time_this_total { time_total };

            // Launch keyline pipeline concurrently with keypoint pipeline
            std::future<std::pair<std::array<map<keyline>, 2>, line3d_cloud>> keyline_future;
            if (_options.use_keylines)
            {
                keyline_future = std::async(std::launch::async, [&]
                {
                    auto kls = _keyline_tracker.track(frame_0, frame_1);
                    auto lns = _keyline_tracker.triangulate(kls, frame_1.undistorted[0]);
                    return std::make_pair(std::move(kls), std::move(lns));
                });
            }

            keypoints = _keypoint_tracker.track(frame_0, frame_1, predicted_pose);
            points3d  = _keypoint_tracker.triangulate(keypoints, frame_1.undistorted[0]);

            if (keyline_future.valid())
            {
                auto [kls, lns] = keyline_future.get();
                keylines = std::move(kls);
                lines3d  = std::move(lns);
            }
        }

        auto ms = [](const auto& duration) { return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count(); };
        SPDLOG_INFO("[TIMING] track() - Total: {}ms", ms(time_total));

        return { frame_1, keypoints, keylines, points3d, lines3d };
    }
} // namespace zenslam
