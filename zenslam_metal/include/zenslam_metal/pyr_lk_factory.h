#pragma once

#include <memory>

#include "zenslam/tracking/pyr_lk.h"

namespace zenslam::metal
{
    [[nodiscard]] auto create_metal_pyr_lk() -> std::shared_ptr<zenslam::pyr_lk>;
}
