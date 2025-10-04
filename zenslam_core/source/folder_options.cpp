#include "folder_options.h"

#include <print>

void zenslam::folder_options::print() const
{
    std::println("folder root: {}", folder_root.string());
    std::println("folder left: {}", folder_left.string());
    std::println("folder right: {}", folder_right.string());
    std::println("folder timescale: {}", folder_timescale);
}
