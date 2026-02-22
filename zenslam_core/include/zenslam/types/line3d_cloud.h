#pragma once

#include <nanoflann.hpp>

#include "keyline.h"
#include "line3d.h"
#include "map.h"

namespace zenslam
{
    class line3d_cloud : public map<line3d>, public nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, line3d_cloud>, line3d_cloud, 3>
    {
    public:
        line3d_cloud();

        line3d_cloud(const line3d_cloud& other);

        auto operator=(const line3d_cloud& other) -> line3d_cloud&;

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
        [[nodiscard]] auto radius_search(const line3d& query_line, double radius) const -> std::vector<line3d>;

        [[nodiscard]] auto size() const -> size_t;

        /**
         * @brief Check if a keyline descriptor matches any landmark in the cloud
         *
         * Uses Hamming distance for binary descriptors (BinaryDescriptor) or L2 for float descriptors.
         * Applies a distance threshold for acceptance.
         *
         * @param keyline Keyline with descriptor to match
         * @param max_descriptor_distance Maximum allowed descriptor distance
         *        For binary descriptors: typically 50-70 bits
         *        For float descriptors: typically 0.3-0.5 normalized distance
         * @return true if a matching landmark is found, false otherwise
         */
        [[nodiscard]] auto match(const keyline& keyline, double max_descriptor_distance = 32.0) const -> std::optional<line3d>;
    };
} // namespace zenslam

inline auto operator*(const cv::Affine3d& pose, const zenslam::line3d_cloud& cloud) -> zenslam::line3d_cloud
{
    zenslam::line3d_cloud cloud_trans { };
    for (const auto& line3d : cloud | std::views::values)
    {
        cloud_trans[line3d.index] = pose * line3d;
    }
    return cloud_trans;
}
