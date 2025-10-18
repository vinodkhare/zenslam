#pragma once

#include <concepts>
#include <cstddef>
#include <map>
#include <vector>

#include <opencv2/core/types.hpp>

namespace zenslam
{
    // Basic concept for checking index field
    template <typename T>
    concept indexable = requires(T obj)
    {
        { obj.index } -> std::same_as<std::size_t>;
    };

    template <indexable T>
    class map
    {
    public:
        // Type aliases for iterators
        using iterator       = std::map<size_t, T>::iterator;
        using const_iterator = std::map<size_t, T>::const_iterator;
        using value_type     = T;
        using key_type       = size_t;

        [[nodiscard]] auto size() const -> size_t;
        [[nodiscard]] auto empty() const -> bool;
        [[nodiscard]] auto contains(size_t index) const -> bool;

        auto at(size_t index) const -> const T &;

        auto operator[](size_t index) const -> const T &;
        auto operator()(size_t index) const -> const T &;

        auto operator+=(const T &item) -> void;
        auto operator+=(const std::vector<T> &items) -> void;
        auto operator+=(const std::map<size_t, T> &other) -> void;
        auto operator+=(const map &other) -> void;
        auto operator*=(const std::vector<cv::DMatch> &matches) -> void;

        // Iterator methods for range-based for loops
        [[nodiscard]] auto begin() -> iterator;
        [[nodiscard]] auto end() -> iterator;
        [[nodiscard]] auto begin() const -> const_iterator;
        [[nodiscard]] auto end() const -> const_iterator;
        [[nodiscard]] auto cbegin() const -> const_iterator;
        [[nodiscard]] auto cend() const -> const_iterator;

    private:
        std::map<size_t, T> _map = { };
    };

    template <indexable T>
    auto map<T>::size() const -> size_t
    {
        return _map.size();
    }

    template <indexable T>
    auto map<T>::empty() const -> bool
    {
        return _map.empty();
    }

    template <indexable T>
    auto map<T>::contains(const size_t index) const -> bool
    {
        return _map.contains(index);
    }

    template <indexable T>
    auto map<T>::at(const size_t index) const -> const T &
    {
        return _map.at(index);
    }

    template <indexable T>
    auto map<T>::operator[](const size_t index) const -> const T &
    {
        return _map[index];
    }

    template <indexable T>
    auto map<T>::operator()(size_t index) const -> const T &
    {
        return _map[index];
    }

    template <indexable T>
    auto map<T>::operator+=(const T &item) -> void
    {
        _map[item.index] = item;
    }

    template <indexable T>
    auto map<T>::operator+=(const std::vector<T> &items) -> void
    {
        for (const auto &item: items)
        {
            _map[item.index] = item;
        }
    }

    template <indexable T>
    auto map<T>::operator+=(const std::map<size_t, T> &other) -> void
    {
        for (const auto &[index, item]: other)
        {
            _map[index] = item;
        }
    }

    template <indexable T>
    auto map<T>::operator+=(const map &other) -> void
    {
        for (const auto &[index, item]: other)
        {
            _map[index] = item;
        }
    }

    template <indexable T>
    auto map<T>::operator*=(const std::vector<cv::DMatch> &matches) -> void
    {
        for (const auto &match: matches)
        {
            if (_map.contains(match.trainIdx))
            {
                auto index_old = match.trainIdx;
                auto index_new = match.queryIdx;

                auto item  = _map.at(index_old);
                item.index = index_new;

                _map.erase(index_old);

                _map[item.index] = item;
            }
        }
    }

    // Iterator implementations
    template <indexable T>
    auto map<T>::begin() -> iterator
    {
        return _map.begin();
    }

    template <indexable T>
    auto map<T>::end() -> iterator
    {
        return _map.end();
    }

    template <indexable T>
    auto map<T>::begin() const -> const_iterator
    {
        return _map.begin();
    }

    template <indexable T>
    auto map<T>::end() const -> const_iterator
    {
        return _map.end();
    }

    template <indexable T>
    auto map<T>::cbegin() const -> const_iterator
    {
        return _map.cbegin();
    }

    template <indexable T>
    auto map<T>::cend() const -> const_iterator
    {
        return _map.cend();
    }
}
