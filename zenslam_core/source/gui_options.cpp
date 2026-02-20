#include "zenslam/gui_options.h"

#include <stdexcept>

namespace zenslam
{
    void gui_options::validate() const
    {
        if (keyline_thickness < 1)
            throw std::invalid_argument("gui.keyline_thickness must be >= 1");
        if (point_cloud_opacity < 0.0f || point_cloud_opacity > 1.0f)
            throw std::invalid_argument("gui.point_cloud_opacity must be in [0.0, 1.0]");
        if (point_size < 1.0f || point_size > 20.0f)
            throw std::invalid_argument("gui.point_size must be in [1.0, 20.0]");
    }
} // namespace zenslam
