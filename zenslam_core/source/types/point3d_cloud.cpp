#include "zenslam/types/point3d_cloud.h"

zenslam::point3d_cloud::point3d_cloud() :
    KDTreeSingleIndexAdaptor
    {
        3,
        *this,
        nanoflann::KDTreeSingleIndexAdaptorParams
        {
            10,
            nanoflann::KDTreeSingleIndexAdaptorFlags::SkipInitialBuildIndex
        }
    }
{
}

zenslam::point3d_cloud::point3d_cloud(const point3d_cloud& other) :
    map(other),
    KDTreeSingleIndexAdaptor
    {
        3,
        *this,
        nanoflann::KDTreeSingleIndexAdaptorParams
        {
            10,
            nanoflann::KDTreeSingleIndexAdaptorFlags::SkipInitialBuildIndex
        }
    }
{
}

auto zenslam::point3d_cloud::operator=(const point3d_cloud& other) -> point3d_cloud&
{
    map::operator=(other);
    return *this;
}

size_t zenslam::point3d_cloud::kdtree_get_point_count() const
{
    return map::size();
}

double zenslam::point3d_cloud::kdtree_get_pt(const size_t idx, const size_t dim) const
{
    if (dim == 0)
        return this->operator()(idx).x;
    if (dim == 1)
        return this->operator()(idx).y;
    return this->operator()(idx).z;
}

auto zenslam::point3d_cloud::radius_search(const point3d& query_point, const double radius) const -> std::vector<point3d>
{
    std::vector<nanoflann::ResultItem<unsigned>> matches;

    const auto count = this->radiusSearch(&query_point.x, radius * radius, matches);

    std::vector<point3d> points_found;
    points_found.reserve(count);

    for (auto i = 0; i < count; i++)
    {
        points_found.emplace_back(this->operator()(i));
    }

    return points_found;
}

auto zenslam::point3d_cloud::size() const -> size_t
{
    return map::size();
}

auto zenslam::point3d_cloud::contains_index(size_t index) const -> bool
{
    return map::contains(index);
}

auto zenslam::point3d_cloud::match(const keypoint& keypoint, const double max_descriptor_distance) const -> std::optional<point3d>
{
    if (keypoint.descriptor.empty())
    {
        return std::nullopt;
    }

    for (const auto& point3d : *this | std::views::values)
    {
        if (point3d.descriptor.empty())
        {
            continue;
        }

        // Compute descriptor distance
        // Use Hamming for binary descriptors (ORB), L2 for float
        auto distance = 0.0;
        if (keypoint.descriptor.type() == CV_8U && point3d.descriptor.type() == CV_8U)
        {
            distance = cv::norm(keypoint.descriptor, point3d.descriptor, cv::NORM_HAMMING);
        }
        else
        {
            // Convert to float if needed for L2 distance
            auto kp_desc = keypoint.descriptor.clone();
            auto lm_desc = point3d.descriptor.clone();
            if (kp_desc.type() != CV_32F)
                kp_desc.convertTo(kp_desc, CV_32F);
            if (lm_desc.type() != CV_32F)
                lm_desc.convertTo(lm_desc, CV_32F);
            distance = cv::norm(kp_desc, lm_desc, cv::NORM_L2);
        }

        if (distance <= max_descriptor_distance)
        {
            return point3d;
        }
    }

    return std::nullopt;
}
