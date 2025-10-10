#pragma once

#include <vector>

#include "pose.h"

namespace zenslam
{
    class groundtruth
    {
    public:
        static groundtruth read(const std::filesystem::path &path);

    private:
        std::vector<pose> poses { };
    };
}
