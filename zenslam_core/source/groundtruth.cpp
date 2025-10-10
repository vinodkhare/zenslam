#include "groundtruth.h"

#include <opencv2/core/quaternion.hpp>

#include <rapidcsv.h>

#include <spdlog/spdlog.h>

zenslam::groundtruth zenslam::groundtruth::read(const std::filesystem::path &path)
{
    groundtruth groundtruth { };

    const rapidcsv::Document doc(path.string(), rapidcsv::LabelParams(0, -1));

    for (size_t i = 0; i < doc.GetRowCount(); i++)
    {
        const auto &timestamp = doc.GetCell<size_t>(0, i) * 1E-9;

        const auto &t = cv::Vec3d
        {
            doc.GetCell<double>(1, i),
            doc.GetCell<double>(2, i),
            doc.GetCell<double>(3, i)
        };

        const auto &q = cv::Quatd
        {
            doc.GetCell<double>(4, i),
            doc.GetCell<double>(5, i),
            doc.GetCell<double>(6, i),
            doc.GetCell<double>(7, i)
        };

        groundtruth.poses.emplace_back(timestamp, cv::Affine3d(q.toRotMat3x3(), t));
    }

    for (auto i = 0; i < groundtruth.poses.size(); i++)
    {
        SPDLOG_TRACE
        (
            "pose: [{}, {}, {}; {}, {}, {}]",
            groundtruth.poses[i].timestamp,
            groundtruth.poses[i].affine.translation()[0],
            groundtruth.poses[i].affine.translation()[1],
            groundtruth.poses[i].affine.translation()[2],
            groundtruth.poses[i].affine.rvec()[0],
            groundtruth.poses[i].affine.rvec()[1],
            groundtruth.poses[i].affine.rvec()[2]
        );
    }

    return groundtruth;
}

