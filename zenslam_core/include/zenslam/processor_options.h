#pragma once

namespace zenslam
{
    class processor_options
    {
    public:
        bool   clahe_enabled         = { };
        double clahe_clip_limit      = { };
        bool   rectification_enabled = { };
    };
}
