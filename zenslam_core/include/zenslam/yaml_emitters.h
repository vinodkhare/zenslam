#pragma once

#include "folder_options.h"
#include "gui_options.h"

namespace zenslam
{
    YAML::Emitter& operator<<(YAML::Emitter& emitter, const folder_options& folder_options);
    YAML::Emitter& operator<<(YAML::Emitter& emitter, const gui_options& gui_options);
}
