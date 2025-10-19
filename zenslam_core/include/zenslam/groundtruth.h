#pragma once

#include <filesystem>
#include <vector>

#include "pose.h"

namespace zenslam
{
    class groundtruth
    {
    public:
        static groundtruth read(const std::filesystem::path& path);

        auto slerp(double timestamp) -> pose;

    private:
        size_t            _index { };
        std::vector<pose> _poses { };
    };
}