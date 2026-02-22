#include "zenslam/detection/keypoint_detector_simple.h"

#include <ranges>

#include <opencv2/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>

zenslam::keypoint_detector_simple::keypoint_detector_simple(const detection_options& options) :
    _options(options)
{
    switch (options.feature_detector)
    {
    case feature_type::FAST:
        _detector = cv::FastFeatureDetector::create(options.fast_threshold);
        break;
    case feature_type::ORB:
        _detector = cv::ORB::create(500, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, options.fast_threshold);
        break;
    case feature_type::SIFT:
        _detector = cv::SIFT::create();
        break;
    }

    switch (options.descriptor)
    {
    case descriptor_type::ORB:
        _describer = cv::ORB::create();
        break;
    case descriptor_type::SIFT:
        _describer = cv::SIFT::create();
        break;
    case descriptor_type::FREAK:
        _describer = cv::xfeatures2d::FREAK::create();
        break;
    }
}

std::vector<zenslam::keypoint> zenslam::keypoint_detector_simple::detect_keypoints(const cv::Mat& image, const map<keypoint>& keypoints_existing) const
{
    // make mask image
    cv::Mat mask { cv::Mat::ones(image.size(), CV_8U) * 255 };
    for (const auto& keypoint : keypoints_existing | std::views::values)
    {
        cv::circle(mask, keypoint.pt, std::min(_options.cell_size.width, _options.cell_size.height) / 2, cv::Scalar(0), -1);
    }

    std::vector<cv::KeyPoint> keypoints_cv { };
    _detector->detect(image, keypoints_cv, mask);

    cv::Mat descriptors { };
    if (!keypoints_cv.empty())
    {
        _describer->compute(image, keypoints_cv, descriptors);
    }

    std::vector<keypoint> keypoints { };
    for (size_t i = 0; i < keypoints_cv.size(); ++i)
    {
        keypoints.emplace_back( keypoints_cv[i], keypoint::index_next++, descriptors.row(static_cast<int>(i)).clone() );
    }

    return keypoints;
}
