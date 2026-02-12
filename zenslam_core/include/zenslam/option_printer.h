#pragma once

#include <spdlog/spdlog.h>

#include "zenslam/formatters.h"
#include "zenslam/option.h"

namespace zenslam
{
    class option_printer
    {
    public:
        template <class T>
        static void print(const option<T>& option)
        {
            SPDLOG_INFO("{}: {}", option.name(), option.value());
        }
    };
}
