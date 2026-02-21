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
        { obj.index } -> std::same_as<std::size_t&>;
    };

    template <indexable T>
    class map : public std::map<size_t, T>
    {
    public:
        /** get all keys that have matching keys in another map
         *
         * @param other another map to match keys against
         * @return a view of all keys with matching keys
         */
        auto keys_matched(const map& other) const -> auto;

        /** get all values
         *
         * @return a view of all values in the map
         */
        auto values() const -> auto;

        /** cast all values to another type
         *
         * @tparam S target type
         * @return a view of all values cast to type S
         */
        template <typename S>
        auto values_cast() const -> auto;

        /** get all values that have matching keys in another map
         *
         * @param other another map to match keys against
         * @return a view of all values with matching keys
         */
        auto values_matched(const map& other) const -> auto;

        /** get all values that do not have matching keys in another map
         *
         * @param other another map to match keys against
         * @return a view of all values without matching keys
         */
        auto values_unmatched(const map& other) const -> auto;

        /** get all values that have matching keys in another map of different type
         *
         * @tparam S type of the other map, S must also be indexable
         * @param other another map to match keys against
         * @return a view of all values with matching keys
         */
        template <indexable S>
        auto values_matched(const map<S>& other) const -> auto;

        /** get all values that do not have matching keys in another map of different type
         *
         * @tparam S type of the other map, S must also be indexable
         * @param other another map to match keys against
         * @return a view of all values without matching keys
         */
        template <indexable S>
        auto values_unmatched(const map<S>& other) const -> auto;

        template <typename T_SLICE>
        [[nodiscard]] auto values_sliced(const std::function<T_SLICE(const T&)>& slice_function) const -> auto;

        // ordered element access
        auto operator()(size_t i) const -> const T&; // get item by insertion order

        // add items
        auto operator+=(const T& item) -> void;                    // add item to map
        auto operator+=(const std::vector<T>& items) -> void;      // add all items to map
        auto operator+=(const std::map<size_t, T>& other) -> void; // add another std::map to this map
        auto operator+=(const map& other) -> void;                 // add another map to this map

        // remap indices
        auto operator*=(const std::vector<cv::DMatch>& matches) -> void; // remap indices based on matches

        void add(const std::vector<T>& items, const bool& overwrite = false);

    private:
        std::vector<size_t> _indices = { };
    };

    template <indexable T>
    auto map<T>::keys_matched(const map& other) const -> auto
    {
        return *this | std::views::keys | std::views::filter
        (
            [&other](const auto& index)
            {
                return other.contains(index);
            }
        ) | std::ranges::to<std::vector>();
    }

    template <indexable T>
    auto map<T>::values() const -> auto
    {
        return *this | std::views::values;
    }

    template <indexable T>
    template <typename S>
    auto map<T>::values_cast() const -> auto
    {
        return *this | std::views::values | std::views::transform
        (
            [](const auto& item) -> S
            {
                return static_cast<S>(item);
            }
        );
    }

    template <indexable T>
    auto map<T>::values_matched(const map& other) const -> auto
    {
        return *this | std::views::values | std::views::filter
        (
            [&other](const auto& item)
            {
                return other.contains(item.index);
            }
        );
    }

    template <indexable T>
    auto map<T>::values_unmatched(const map& other) const -> auto
    {
        return *this | std::views::values | std::views::filter
        (
            [&other](const auto& item)
            {
                return !other.contains(item.index);
            }
        );
    }

    template <indexable T>
    template <indexable S>
    auto map<T>::values_matched(const map<S>& other) const -> auto
    {
        return *this | std::views::values | std::views::filter
        (
            [&other](const auto& item)
            {
                return other.contains(item.index);
            }
        );
    }

    template <indexable T>
    template <indexable S>
    auto map<T>::values_unmatched(const map<S>& other) const -> auto
    {
        return *this | std::views::values | std::views::filter
        (
            [&other](const auto& item)
            {
                return !other.contains(item.index);
            }
        );
    }

    template <indexable T>
    template <typename T_SLICE>
    auto map<T>::values_sliced(const std::function<T_SLICE(const T&)>& slice_function) const -> auto
    {
        return *this | std::views::values | std::views::transform(slice_function);
    }

    template <indexable T>
    auto map<T>::operator()(const size_t i) const -> const T&
    {
        return this->at(this->_indices[i]);
    }

    template <indexable T>
    auto map<T>::operator+=(const T& item) -> void
    {
        if (!this->contains(item.index))
            this->_indices.push_back(item.index);
        this->operator[](item.index) = item;
    }

    template <indexable T>
    auto map<T>::operator+=(const std::vector<T>& items) -> void
    {
        for (const auto& item : items)
        {
            if (!this->contains(item.index))
                this->_indices.push_back(item.index);
            this->operator[](item.index) = item;
        }
    }

    template <indexable T>
    auto map<T>::operator+=(const std::map<size_t, T>& other) -> void
    {
        for (const auto& [index, item] : other)
        {
            if (!this->contains(item.index))
                this->_indices.push_back(item.index);
            this->operator[](item.index) = item;
        }
    }

    template <indexable T>
    auto map<T>::operator+=(const map& other) -> void
    {
        for (const auto& [index, item] : other)
        {
            if (!this->contains(item.index))
            {
                this->_indices.push_back(item.index);
                this->operator[](item.index) = item;
            }
        }
    }

    template <indexable T>
    auto map<T>::operator*=(const std::vector<cv::DMatch>& matches) -> void
    {
        for (const auto& match : matches)
        {
            if (this->contains(match.trainIdx))
            {
                auto index_old = match.trainIdx;
                auto index_new = match.queryIdx;

                auto item  = this->at(index_old);
                item.index = index_new;

                this->erase(index_old);

                this->operator[](item.index) = item;
            }
        }

        this->_indices.clear();
        for (const auto& [index, item] : *this)
        {
            this->_indices.push_back(index);
        }
    }

    template <indexable T>
    void map<T>::add(const std::vector<T>& items, const bool& overwrite)
    {
        for (const auto& item : items)
        {
            if (overwrite || !this->contains(item.index))
            {
                (*this)[item.index] = item;
            }
        }
    }
}
