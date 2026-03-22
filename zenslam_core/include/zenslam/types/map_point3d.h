#pragma once

#include <set>

#include "point3d.h"

namespace zenslam
{
    class map_point3d : public point3d
    {
    public:
        std::set<size_t> visibility;
    };
}
