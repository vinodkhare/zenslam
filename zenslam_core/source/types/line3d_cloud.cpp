#include "zenslam/types/line3d_cloud.h"

zenslam::line3d_cloud::line3d_cloud() :
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

zenslam::line3d_cloud::line3d_cloud(const line3d_cloud& other) :
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

auto zenslam::line3d_cloud::operator=(const line3d_cloud& other) -> line3d_cloud&
{
    map::operator=(other);
    return *this;
}

size_t zenslam::line3d_cloud::kdtree_get_point_count() const
{
    return map::size();
}

double zenslam::line3d_cloud::kdtree_get_pt(const size_t idx, const size_t dim) const
{
    // Use midpoint of the line segment for KD-tree queries
    const auto& line = this->operator()(idx);
    const auto midpoint_x = (line[0].x + line[1].x) / 2.0;
    const auto midpoint_y = (line[0].y + line[1].y) / 2.0;
    const auto midpoint_z = (line[0].z + line[1].z) / 2.0;

    if (dim == 0)
        return midpoint_x;
    if (dim == 1)
        return midpoint_y;
    return midpoint_z;
}

auto zenslam::line3d_cloud::radius_search(const line3d& query_line, const double radius) const -> std::vector<line3d>
{
    // Use midpoint of query line for radius search
    const auto midpoint_x = (query_line[0].x + query_line[1].x) / 2.0;
    const auto midpoint_y = (query_line[0].y + query_line[1].y) / 2.0;
    const auto midpoint_z = (query_line[0].z + query_line[1].z) / 2.0;

    std::vector<nanoflann::ResultItem<unsigned>> matches;

    const double coords[] = {midpoint_x, midpoint_y, midpoint_z};
    const auto count = this->radiusSearch(coords, radius * radius, matches);

    std::vector<line3d> lines_found;
    lines_found.reserve(count);

    for (auto i = 0; i < count; i++)
    {
        lines_found.emplace_back(this->operator()(i));
    }

    return lines_found;
}

auto zenslam::line3d_cloud::size() const -> size_t
{
    return map::size();
}

auto zenslam::line3d_cloud::match(const keyline& keyline, const double max_descriptor_distance) const -> std::optional<line3d>
{
    if (keyline.descriptor.empty())
    {
        return std::nullopt;
    }

    for (const auto& line3d : *this | std::views::values)
    {
        if (line3d.descriptor.empty())
        {
            continue;
        }

        // Compute descriptor distance
        // Use Hamming for binary descriptors (BinaryDescriptor), L2 for float
        auto distance = 0.0;
        if (keyline.descriptor.type() == CV_8U && line3d.descriptor.type() == CV_8U)
        {
            distance = cv::norm(keyline.descriptor, line3d.descriptor, cv::NORM_HAMMING);
        }
        else
        {
            // Convert to float if needed for L2 distance
            auto kl_desc = keyline.descriptor.clone();
            auto lm_desc = line3d.descriptor.clone();
            if (kl_desc.type() != CV_32F)
                kl_desc.convertTo(kl_desc, CV_32F);
            if (lm_desc.type() != CV_32F)
                lm_desc.convertTo(lm_desc, CV_32F);
            distance = cv::norm(kl_desc, lm_desc, cv::NORM_L2);
        }

        if (distance <= max_descriptor_distance)
        {
            return line3d;
        }
    }

    return std::nullopt;
}
