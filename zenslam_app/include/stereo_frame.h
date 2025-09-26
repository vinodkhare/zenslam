#pragma once

#include "mono_frame.h"

namespace zenslam
{
    class stereo_frame
    {
    public:
        mono_frame l  = mono_frame();
        mono_frame r = mono_frame();

        stereo_frame() = default;

        stereo_frame(const mono_frame &l, const mono_frame &r) : l(l), r(r)
        {
        }
    };
} // namespace zenslam
