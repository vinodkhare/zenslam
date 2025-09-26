#pragma once

#include "mono_frame.h"

namespace zenslam
{
    class stereo_frame
    {
    public:
        mono_frame left  = mono_frame();
        mono_frame right = mono_frame();

        stereo_frame() = default;

        stereo_frame(const mono_frame &l, const mono_frame &r) : left(l), right(r)
        {
        }
    };
} // namespace zenslam
