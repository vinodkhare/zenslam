#pragma once

#include <nanoflann.hpp>

#include "keypoint.h"
#include "map.h"
#include "point3d.h"

namespace zenslam
{
    class point3d_cloud : public map<point3d>, public nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, point3d_cloud>, point3d_cloud, 3>
    {
    public:
        point3d_cloud();

        point3d_cloud(const point3d_cloud& other);

        auto operator=(const point3d_cloud& other) -> point3d_cloud&;

        // Must have these methods for nanoflann
        [[nodiscard]] size_t kdtree_get_point_count() const;

        [[nodiscard]] double kdtree_get_pt(size_t idx, size_t dim) const;

        template <class BBOX>
        // ReSharper disable once CppMemberFunctionMayBeStatic
        bool kdtree_get_bbox(BBOX&) const
        {
            return false;
        }

        // Additional helper methods
        [[nodiscard]] auto radius_search(const point3d& query_point, double radius) const -> std::vector<point3d>;

        [[nodiscard]] auto size() const -> size_t;

        /**
         * Return true if a point3d with the given index exists in the cloud, false otherwise
         * @param index Index of the point3d to check for existence in the cloud
         * @return true if a point3d with the given index exists in the cloud, false otherwise
         */
        [[nodiscard]] auto contains_index(size_t index) const -> bool;

        /**
         * @brief Check if a keypoint descriptor matches any landmark in the cloud
         *
         * Uses Hamming distance for binary descriptors (ORB) or L2 for float descriptors.
         * Applies a distance threshold for acceptance.
         *
         * @param keypoint Keypoint with descriptor to match
         * @param max_descriptor_distance Maximum allowed descriptor distance
         *        For ORB binary: typically 50-70 bits
         *        For float descriptors: typically 0.3-0.5 normalized distance
         * @return true if a matching landmark is found, false otherwise
         */
        [[nodiscard]] auto match(const keypoint& keypoint, double max_descriptor_distance = 32.0) const -> std::optional<point3d>;
    };
} // namespace zenslam


inline auto operator*(const cv::Affine3d& pose, const zenslam::point3d_cloud& cloud) -> zenslam::point3d_cloud
{
    zenslam::point3d_cloud cloud_trans { };
    for (const auto& point3d : cloud | std::views::values)
    {
        cloud_trans[point3d.index]            = pose * point3d;
        cloud_trans[point3d.index].index      = point3d.index;
        cloud_trans[point3d.index].descriptor = point3d.descriptor;
    }
    return cloud_trans;
}
