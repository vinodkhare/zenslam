#pragma once

#include <nanoflann.hpp>
#include <opencv2/core/mat.hpp>

#include "keypoint.h"
#include "map.h"
#include "point3d.h"

namespace zenslam
{
    class point3d_cloud : public map<point3d>, public nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, point3d_cloud>, point3d_cloud, 3>
    {
    public:
        point3d_cloud() :
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

        point3d_cloud(const point3d_cloud& other) :
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

        auto operator=(const point3d_cloud& other) -> point3d_cloud&
        {
            map::operator=(other);
            return *this;
        }

        // Must have these methods for nanoflann
        [[nodiscard]] size_t kdtree_get_point_count() const
        {
            return map::size();
        }

        [[nodiscard]] double kdtree_get_pt(const size_t idx, const size_t dim) const
        {
            if (dim == 0)
                return this->operator()(idx).x;
            if (dim == 1)
                return this->operator()(idx).y;
            return this->operator()(idx).z;
        }

        template <class BBOX>
        // ReSharper disable once CppMemberFunctionMayBeStatic
        bool kdtree_get_bbox(BBOX&) const
        {
            return false;
        }

        // Additional helper methods
        auto radius_search(const point3d& query_point, const double radius) const -> std::vector<point3d>
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

        [[nodiscard]] auto size() const -> size_t
        {
            return map::size();
        }

        /**
         * @brief Check if a keypoint descriptor matches any landmark in the cloud
         *
         * Uses Hamming distance for binary descriptors (ORB) or L2 for float descriptors.
         * Applies a distance threshold for acceptance.
         *
         * @param kp Keypoint with descriptor to match
         * @param max_descriptor_distance Maximum allowed descriptor distance
         *        For ORB binary: typically 50-70 bits
         *        For float descriptors: typically 0.3-0.5 normalized distance
         * @return true if a matching landmark is found, false otherwise
         */
        [[nodiscard]] auto match(
            const keypoint& keypoint,
            double max_descriptor_distance = 50.0) const -> std::optional<point3d>
        {
            if (keypoint.descriptor.empty())
            {
                return std::nullopt;
            }

            for (const auto& [_, point3d] : *this)
            {
                if (point3d.descriptor.empty())
                {
                    continue;
                }

                // Compute descriptor distance
                // Use Hamming for binary descriptors (ORB), L2 for float
                double distance = 0.0;
                if (keypoint.descriptor.type() == CV_8U && point3d.descriptor.type() == CV_8U)
                {
                    distance = cv::norm(keypoint.descriptor, point3d.descriptor, cv::NORM_HAMMING);
                }
                else
                {
                    // Convert to float if needed for L2 distance
                    cv::Mat kp_desc = keypoint.descriptor.clone();
                    cv::Mat lm_desc = point3d.descriptor.clone();
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
    };
}