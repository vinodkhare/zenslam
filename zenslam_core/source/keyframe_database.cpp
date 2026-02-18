#include "zenslam/keyframe_database.h"

#include <algorithm>

namespace zenslam
{
    keyframe_database::keyframe_database(const size_t min_shared_features) :
        _min_shared_features(min_shared_features)
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

    auto keyframe_database::contains(const size_t id) const -> bool
    {
        return _keyframes.contains(id);
    }

    auto keyframe_database::get(const size_t id) const -> frame::estimated
    {
        if (_keyframes.contains(id))
        {
            return _keyframes.at(id);
        }

        throw std::out_of_range("Keyframe ID not found in database");
    }

    auto keyframe_database::last() const -> frame::estimated
    {
        if (_order.empty())
        {
            throw std::runtime_error("Keyframe database is empty");
        }

        return get(_order.back());
    }

    auto keyframe_database::recent(const size_t max_count) const -> std::vector<frame::estimated>
    {
        std::vector<frame::estimated> result;
        if (_order.empty() || max_count == 0)
        {
            return result;
        }

        const size_t start = _order.size() > max_count ? _order.size() - max_count : 0;
        result.reserve(_order.size() - start);
        for (size_t i = start; i < _order.size(); ++i)
        {
            result.push_back(get(_order[i]));
        }

        return result;
    }

    auto keyframe_database::add(const frame::estimated& frame) -> const frame::estimated&
    {
        if (const auto it = _keyframes.find(frame.index); it != _keyframes.end())
        {
            return it->second;
        }

        for (const auto& [id, other] : _keyframes)
        {
            const size_t shared = compute_shared(frame, other);

            if (shared >= _min_shared_features)
            {
                _covisibility[frame.index][id] = shared;
                _covisibility[id][frame.index] = shared;
            }
        }

        _keyframes[frame.index] = frame;
        _order.push_back(frame.index);

        return _keyframes.at(frame.index);
    }

    auto keyframe_database::update_pose(const size_t id, const cv::Affine3d& pose) -> void
    {
        if (_keyframes.contains(id))
        {
            _keyframes.at(id).pose = pose;
        }

        throw std::out_of_range("Keyframe ID not found in database");
    }

    auto keyframe_database::covisible(const size_t id) const -> std::vector<std::pair<size_t, size_t>>
    {
        std::vector<std::pair<size_t, size_t>> result;

        if (_covisibility.contains(id))
        {
            result.reserve(_covisibility.at(id).size());
            for (const auto& [other_id, shared] : _covisibility.at(id))
            {
                result.emplace_back(other_id, shared);
            }

            std::ranges::sort(result, [](const auto& a, const auto& b)
            {
                return a.second > b.second;
            });
        }

        return result;
    }

    auto keyframe_database::compute_shared(const frame::estimated& a, const frame::estimated& b) -> size_t
    {
        const size_t shared_points = a.keypoints[0].keys_matched(b.keypoints[0]).size();
        const size_t shared_lines  = a.keylines[0].keys_matched(b.keylines[0]).size();
        return shared_points + shared_lines;
    }
}
