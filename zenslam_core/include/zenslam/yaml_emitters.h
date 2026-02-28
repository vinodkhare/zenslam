#pragma once

#include "folder_options.h"

namespace zenslam
{
    YAML::Emitter& operator<<(YAML::Emitter& emitter, const folder_options& folder_options);
}
