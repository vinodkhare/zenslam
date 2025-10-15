#pragma once

namespace zenslam {
    enum class feature_type {
        FAST,
        ORB,
        SIFT
    };

    enum class descriptor_type {
        ORB,
        SIFT,
        FREAK
    };
}