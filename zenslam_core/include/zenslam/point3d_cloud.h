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

        // Must have these methods for nanoflann
        size_t kdtree_get_point_count() const
        {
            return map::size();
        }

        double kdtree_get_pt(const size_t idx, const size_t dim) const
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
    };
}
