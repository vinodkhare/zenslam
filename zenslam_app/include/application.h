#pragma once

#include <opencv2/core.hpp>

#include "slam_thread.h"
#include "stereo_frame.h"
#include "thread_safe.h"

namespace zenslam
{
    class application
    {
    public:
        explicit application(const options &options);

        void render();

    private:
        thread_safe<stereo_frame> _frame { };

        options     _options { };
        slam_thread _slam_thread { _options };
    };
}
