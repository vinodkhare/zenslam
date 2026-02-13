#pragma once

#include <spdlog/spdlog.h>
#include <opencv2/core/types.hpp>

#include "zenslam/option.h"
#include "zenslam/formatters.h"

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

        /** Specialization for cv::Size */
        static void print(const option<cv::Size>& option)
        {
            SPDLOG_INFO("{}: [{}, {}]", option.name(), option.value().width, option.value().height);
        }

        /** Specialization for cv::Scalar */
        static void print(const option<cv::Scalar>& option)
        {
            const auto& val = option.value();
            SPDLOG_INFO("{} (BGR): [{}, {}, {}]", option.name(), val[0], val[1], val[2]);
        }
    };
}
