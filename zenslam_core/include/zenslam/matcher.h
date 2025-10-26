#pragma once

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include "zenslam/options.h"
#include "zenslam/types/keypoint.h"
#include "zenslam/types/map.h"

namespace zenslam
{
    /** Matcher for keypoints between frames using their descriptors.
     */
    class matcher
    {
    public:
        matcher(const class options::slam& opts, bool is_binary);

        /** Match keypoints between two frames using their descriptors.
         *
         * @param keypoints_0 Keypoints from frame 0
         * @param keypoints_1 Keypoints from frame 1
         * @return Vector of cv::DMatch representing the matched keypoints
         */
        [[nodiscard]] auto match_keypoints(const map<keypoint>& keypoints_0, const map<keypoint>& keypoints_1) const -> std::vector<cv::DMatch>;

    private:
        class options::slam            _options = { };
        cv::Ptr<cv::DescriptorMatcher> _matcher = { };
    };
}
