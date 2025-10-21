#pragma once

#include <nanoflann.hpp>

#include "map.h"
#include "point3d.h"

namespace zenslam
{
    class point3d_cloud : public map<point3d>, public nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, point3d_cloud>, point3d_cloud, 3>
    {
    public:
        point3d_cloud() :
            KDTreeSingleIndexAdaptor { 3, *this } {}

        point3d_cloud(const point3d_cloud& other) :
            map(other),
            KDTreeSingleIndexAdaptor { 3, *this } {}

        auto operator=(const point3d_cloud& other) -> point3d_cloud&
        {
            // copy
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
            if (dim == 0) return this->operator()(idx).x;
            if (dim == 1) return this->operator()(idx).y;
            return this->operator()(idx).z;
        }

        template <class BBOX>
        // ReSharper disable once CppMemberFunctionMayBeStatic
        bool kdtree_get_bbox(BBOX&) const
        {
            return false;
        }

        [[nodiscard]] auto size() const -> size_t
        {
            return map::size();
        }
    };
}
