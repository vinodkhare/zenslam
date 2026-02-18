#include "zenslam/mapping/keyframe_database.h"

#include <algorithm>

namespace zenslam
{
    keyframe_database::keyframe_database(size_t min_shared_features)
        : _min_shared_features(min_shared_features)
    {
    }

    void keyframe_database::clear()
    {
        _keyframes.clear();
        _order.clear();
        _covisibility.clear();
    }

    auto keyframe_database::empty() const -> bool
    {
        return _keyframes.empty();
    }

    auto keyframe_database::size() const -> size_t
    {
        return _keyframes.size();
    }

    auto keyframe_database::contains(size_t id) const -> bool
    {
        return _keyframes.contains(id);
    }

    auto keyframe_database::get(size_t id) const -> const frame::estimated*
    {
        if (const auto it = _keyframes.find(id); it != _keyframes.end())
        {
            return &it->second;
        }
        return nullptr;
    }

    auto keyframe_database::last() const -> const frame::estimated*
    {
        if (_order.empty())
        {
            return nullptr;
        }

        return get(_order.back());
    }

    auto keyframe_database::recent(size_t max_count) const -> std::vector<const frame::estimated*>
    {
        std::vector<const frame::estimated*> result;
        if (_order.empty() || max_count == 0)
        {
            return result;
        }

        const size_t start = _order.size() > max_count ? _order.size() - max_count : 0;
        result.reserve(_order.size() - start);
        for (size_t i = start; i < _order.size(); ++i)
        {
            if (const auto* entry = get(_order[i]))
            {
                result.push_back(entry);
            }
        }
        return result;
    }

    auto keyframe_database::add(const frame::estimated& frame) -> const frame::estimated&
    {
        if (const auto it = _keyframes.find(frame.index); it != _keyframes.end())
        {
            return it->second;
        }

        const frame::estimated entry = frame;

        for (const auto& [id, other] : _keyframes)
        {
            const size_t shared = compute_shared(entry, other);
            if (shared >= _min_shared_features)
            {
                _covisibility[entry.index][id] = shared;
                _covisibility[id][entry.index] = shared;
            }
        }

        _keyframes[entry.index] = entry;
        _order.push_back(frame.index);

        return _keyframes.at(frame.index);
    }

    auto keyframe_database::update_pose(const size_t id, const cv::Affine3d& pose) -> bool
    {
        if (auto it = _keyframes.find(id); it != _keyframes.end())
        {
            it->second.pose = pose;
            return true;
        }

        return false;
    }

    auto keyframe_database::covisible(size_t id) const -> std::vector<std::pair<size_t, size_t>>
    {
        std::vector<std::pair<size_t, size_t>> result;
        if (const auto it = _covisibility.find(id); it != _covisibility.end())
        {
            result.reserve(it->second.size());
            for (const auto& [other_id, shared] : it->second)
            {
                result.emplace_back(other_id, shared);
            }

            std::sort(result.begin(), result.end(), [](const auto& a, const auto& b)
            {
                return a.second > b.second;
            });
        }
        return result;
    }

    auto keyframe_database::compute_shared(const frame::estimated& a, const frame::estimated& b) const -> size_t
    {
        const size_t shared_points = a.keypoints[0].keys_matched(b.keypoints[0]).size();
        const size_t shared_lines = a.keylines[0].keys_matched(b.keylines[0]).size();
        return shared_points + shared_lines;
    }
}
