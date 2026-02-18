#pragma once

#include <cstddef>
#include <unordered_map>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

#include "zenslam/frame/estimated.h"
namespace zenslam
{
    class keyframe_database
    {
    public:
        explicit keyframe_database(size_t min_shared_features = 15);

        void clear();

        [[nodiscard]] auto empty() const -> bool;
        [[nodiscard]] auto size() const -> size_t;
        [[nodiscard]] auto contains(size_t id) const -> bool;
        [[nodiscard]] auto get(size_t id) const -> const frame::estimated*;
        [[nodiscard]] auto last() const -> const frame::estimated*;
        [[nodiscard]] auto recent(size_t max_count) const -> std::vector<const frame::estimated*>;

        auto add(const frame::estimated& frame) -> const frame::estimated&;
        auto update_pose(size_t id, const cv::Affine3d& pose) -> bool;
        [[nodiscard]] auto covisible(size_t id) const -> std::vector<std::pair<size_t, size_t>>;

    private:
        size_t _min_shared_features = 0;
        std::unordered_map<size_t, frame::estimated> _keyframes = { };
        std::vector<size_t> _order = { };
        std::unordered_map<size_t, std::unordered_map<size_t, size_t>> _covisibility = { };

        auto compute_shared(const frame::estimated& a, const frame::estimated& b) const -> size_t;
    };
}
