#include "zenslam/groundtruth.h"

#include <opencv2/core/quaternion.hpp>

#include <rapidcsv.h>

zenslam::groundtruth zenslam::groundtruth::read(const std::filesystem::path& path)
{
    groundtruth groundtruth { };

    const rapidcsv::Document doc(path.string(), rapidcsv::LabelParams(0, -1));

    for (size_t i = 0; i < doc.GetRowCount(); i++)
    {
        const auto& timestamp = doc.GetCell<size_t>(0, i) * 1E-9;

        const auto& t = cv::Vec3d
        {
            doc.GetCell<double>(1, i),
            doc.GetCell<double>(2, i),
            doc.GetCell<double>(3, i)
        };

        const auto& q = cv::Quatd
        {
            doc.GetCell<double>(4, i),
            doc.GetCell<double>(5, i),
            doc.GetCell<double>(6, i),
            doc.GetCell<double>(7, i)
        };

        groundtruth._poses.emplace_back(timestamp, t, q);
    }

    return groundtruth;
}

auto zenslam::groundtruth::slerp(const double timestamp) -> pose
{
    while (_index < _poses.size() && _poses[_index].timestamp < timestamp)
        _index++;

    const auto& t = (timestamp - _poses[_index - 1].timestamp) /
        (_poses[_index].timestamp - _poses[_index - 1].timestamp);

    pose pose { };
    pose.translation = _poses[_index - 1].translation * (1.0 - t) + _poses[_index].translation * t;
    pose.quaternion  = cv::Quatd::slerp(_poses[_index - 1].quaternion, _poses[_index].quaternion, t);

    return pose;
}