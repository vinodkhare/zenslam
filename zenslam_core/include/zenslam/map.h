#pragma once

#include <concepts>
#include <cstddef>
#include <map>
#include <ranges>
#include <vector>

#include <opencv2/core/types.hpp>

namespace zenslam
{
    // Basic concept for checking index field
    template <typename T>
    concept indexable = requires(T obj)
    {
        { obj.index } -> std::same_as<std::size_t &>;
    };

    template <indexable T>
    class map : public std::map<size_t, T>
    {
    public:
        // Type aliases for iterators
        using iterator       = std::map<size_t, T>::iterator;
        using const_iterator = std::map<size_t, T>::const_iterator;
        using value_type     = T;
        using key_type       = size_t;

        static auto create(const std::vector<size_t> &indices, const std::vector<T> &items) -> map;

        auto keys_matched(const map &other) const -> auto;
        auto values() const -> auto;
        auto values_matched(const map &other) const -> auto;

        template <indexable S>
        auto values_matched(const map<S> &other) const -> auto;

        template <typename S>
        auto values_cast() const -> auto;

        auto operator+=(const T &item) -> void;
        auto operator+=(const std::vector<T> &items) -> void;
        auto operator+=(const std::map<size_t, T> &other) -> void;
        auto operator+=(const map &other) -> void;
        auto operator*=(const std::vector<cv::DMatch> &matches) -> void;
    };

    template <indexable T>
    auto map<T>::create(const std::vector<size_t> &indices, const std::vector<T> &items) -> map
    {
        map result;

        for (size_t i = 0; i < indices.size(); ++i)
        {
            result._map[indices[i]] = items[i];
        }

        return result;
    }

    template <indexable T>
    auto map<T>::keys_matched(const map &other) const -> auto
    {
        return *this | std::views::keys | std::views::filter
               (
                   [&other](const auto &index)
                   {
                       return other.contains(index);
                   }
               );
    }

    template <indexable T>
    auto map<T>::values() const -> auto
    {
        return *this | std::views::values;
    }

    template <indexable T>
    auto map<T>::values_matched(const map &other) const -> auto
    {
        return *this | std::views::values | std::views::filter
               (
                   [&other](const auto &item)
                   {
                       return other.contains(item.index);
                   }
               );
    }

    template <indexable T>
    template <indexable S>
    auto map<T>::values_matched(const map<S> &other) const -> auto
    {
        return *this | std::views::values | std::views::filter
               (
                   [&other](const auto &item)
                   {
                       return other.contains(item.index);
                   }
               );
    }

    template <indexable T>
    template <typename S>
    auto map<T>::values_cast() const -> auto
    {
        return *this | std::views::values | std::views::transform
               (
                   [](const auto &item) -> S
                   {
                       return static_cast<S>(item);
                   }
               );
    }


    template <indexable T>
    auto map<T>::operator+=(const T &item) -> void
    {
        (*this)[item.index] = item;
    }

    template <indexable T>
    auto map<T>::operator+=(const std::vector<T> &items) -> void
    {
        for (const auto &item: items)
        {
            (*this)[item.index] = item;
        }
    }

    template <indexable T>
    auto map<T>::operator+=(const std::map<size_t, T> &other) -> void
    {
        for (const auto &[index, item]: other)
        {
            (*this)[index] = item;
        }
    }

    template <indexable T>
    auto map<T>::operator+=(const map &other) -> void
    {
        for (const auto &[index, item]: other)
        {
            (*this)[index] = item;
        }
    }

    template <indexable T>
    auto map<T>::operator*=(const std::vector<cv::DMatch> &matches) -> void
    {
        for (const auto &match: matches)
        {
            if (this->contains(match.trainIdx))
            {
                auto index_old = match.trainIdx;
                auto index_new = match.queryIdx;

                auto item  = this->at(index_old);
                item.index = index_new;

                this->erase(index_old);

                (*this)[item.index] = item;
            }
        }
    }
}
