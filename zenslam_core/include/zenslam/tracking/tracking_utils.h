#pragma once

#include <vector>

#include <opencv2/core/mat.hpp>

#include "zenslam/all_options.h"
#include "zenslam/types/keyline.h"
#include "zenslam/types/map.h"

namespace zenslam::utils
{
    /** Track keylines from frame_0 to frame_1 using KLT optical flow on endpoints.
     *
     * This function tracks keylines by tracking their two endpoints independently using KLT.
     * Forward-backward tracking is used to filter out bad tracks. A keyline is considered
     * successfully tracked only if both endpoints are successfully tracked and pass the
     * forward-backward error check.
     *
     * @param pyramid_0 The image pyramid of the first frame.
     * @param pyramid_1 The image pyramid of the second frame.
     * @param keylines_map_0 A map of keylines in the first frame to be tracked.
    * @param options SLAM options that include tracking KLT parameters.
     * @return A vector of tracked keylines in frame_1.
     */
    auto track_keylines(
        const std::vector<cv::Mat>& pyramid_0,
        const std::vector<cv::Mat>& pyramid_1,
        const map<keyline>&         keylines_map_0,
        const slam_options&         options) -> std::vector<keyline>;
}
