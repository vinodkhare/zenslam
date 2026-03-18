#pragma once

#include <memory>

#include "zenslam/tracking/pyr_lk.h"

namespace zenslam::metal
{
    auto create_metal_pyr_lk() -> std::shared_ptr<zenslam::pyr_lk>;
}
