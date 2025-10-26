#include "zenslam/tracker.h"

#include <thread>
#include <utility>

#include <opencv2/core.hpp>

#include "zenslam/detector.h"
#include "zenslam/utils_slam.h"

namespace zenslam
{
    tracker::tracker(calibration  calib, class options::slam  opts)
        : _calibration(std::move(calib)), _options(std::move(opts))
    {
        const bool is_binary = _options.descriptor == descriptor_type::ORB || _options.descriptor == descriptor_type::FREAK;
        _matcher             = utils::create_matcher(_options, is_binary);
        _detector            = detector::create(_options);
    }

    auto tracker::track(const frame::tracked& frame_0, const frame::processed& frame_1) const -> frame::tracked
    {
        map<keypoint> keypoints_0 = { };
        map<keypoint> keypoints_1 = { };
        map<point3d>  points3d    = { };

        {
            std::jthread thread_0 {
                [&]()
                {
                    keypoints_0 += utils::track_keypoints
                    (
                        frame_0.pyramids[0],
                        frame_1.pyramids[0],
                        frame_0.keypoints[0],
                        _options,
                        _calibration.camera_matrix[0]
                    );

                    if (_options.use_parallel_detector)
                    {
                        keypoints_0 += _detector.detect_keypoints_par(frame_1.undistorted[0], keypoints_0);
                    }
                    else
                    {
                        keypoints_0 += _detector.detect_keypoints(frame_1.undistorted[0], keypoints_0);
                    }
                }
            };

            std::jthread thread_1 {
                [&]()
                {
                    keypoints_1 += utils::track_keypoints
                    (
                        frame_0.pyramids[1],
                        frame_1.pyramids[1],
                        frame_0.keypoints[1],
                        _options,
                        _calibration.camera_matrix[1]
                    );

                    if (_options.use_parallel_detector)
                    {
                        keypoints_1 += _detector.detect_keypoints_par(frame_1.undistorted[1], keypoints_1);
                    }
                    else
                    {
                        keypoints_1 += _detector.detect_keypoints(frame_1.undistorted[1], keypoints_1);
                    }
                }
            };
        }

        keypoints_1 *= utils::match_keypoints(keypoints_0, keypoints_1, _matcher, _options);

        points3d += utils::triangulate_keypoints
        (
            keypoints_0,
            keypoints_1,
            _calibration.projection_matrix[0],
            _calibration.projection_matrix[1],
            _options.triangulation_reprojection_threshold,
            _calibration.cameras[1].pose_in_cam0.translation()
        );

        return { frame_1, keypoints_0, keypoints_1, { }, points3d };
    }
}
