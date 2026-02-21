#pragma once

#include <cmath>
#include <map>
#include <numeric>
#include <ranges>
#include <vector>
#include <ranges>
#include <algorithm>

// This namespace contains utility functions that depend only on the std:: namespace
namespace zenslam::utils
{
    // TEMPLATES
    template <typename T_OUT, typename T_IN>
    auto cast(const std::vector<T_IN>& values) -> std::vector<T_OUT>
    {
        return values | std::views::transform([](const T_IN& value) { return static_cast<T_OUT>(value); }) | std::ranges::to<std::vector>();
    }

    template <typename T_KEY, typename T_VALUE>
    auto invert(const std::map<T_KEY, T_VALUE>& map) -> std::map<T_VALUE, T_KEY>
    {
        return map | std::views::transform([](const auto& pair) { return std::make_pair(pair.second, pair.first); }) | std::ranges::to<std::map>();
    }

    // Generic template function to join any range of elements into a string
    template <std::ranges::input_range Range, typename Projection = std::identity>
        requires std::invocable<Projection, std::ranges::range_reference_t<Range>>
    auto join_to_string(const Range& range, const std::string& delimiter, Projection projection = {}) -> std::string
    {
        if (std::ranges::empty(range))
        {
            return "";
        }

        auto transformed = range | std::ranges::views::transform(projection);

        return std::accumulate(
            std::next(transformed.begin()),
            transformed.end(),
            std::string(std::invoke(projection, *std::ranges::begin(range))),
            [&delimiter](const std::string& a, const auto& b) { return a + delimiter + std::string(b); });
    }

    /**
     * Returns mean of a vector
     * @tparam T input type
     * @param values input values
     * @return mean
     */
    template <typename T>
    auto mean(const std::vector<T>& values) -> double
    {
        if (values.empty())
            return 0.0;
        double sum = 0.0;
        for (const auto& v : values)
            sum += static_cast<double>(v);
        return sum / static_cast<double>(values.size());
    }

    /**
     * Returns median of a vector
     * @tparam T arithmetic type
     * @param v vector of values
     * @return median value
     */
    template <typename T>
        requires std::is_arithmetic_v<T>
    auto median(std::vector<T> v) -> double
    {
        if (v.empty())
        {
            return 0.0;
        }

        std::sort(v.begin(), v.end());

        if (const size_t n = v.size(); n % 2 == 1)
        {
            return static_cast<double>(v[n / 2]);
        }
        else
        {
            return (static_cast<double>(v[n / 2 - 1]) + static_cast<double>(v[n / 2])) / 2.0;
        }
    }

    /**
     * Returns standard deviation of a vector
     * @tparam T input type
     * @param values input values
     * @return standard deviation
     */
    template <typename T>
    auto std_dev(const std::vector<T>& values) -> double
    {
        if (values.size() < 2)
        {
            return 0.0;
        }

        const double m      = mean(values);
        double       sum_sq = 0.0;

        for (const auto& v : values)
        {
            const double diff = static_cast<double>(v) - m;
            sum_sq += diff * diff;
        }

        return std::sqrt(sum_sq / static_cast<double>(values.size() - 1));
    }

    template <typename T_KEY, typename T_VALUE>
    auto values(const std::map<T_KEY, T_VALUE>& map) -> std::vector<T_VALUE>
    {
        return map | std::views::values | std::ranges::to<std::vector>();
    }

    template <size_t N>
    auto to_string(const std::array<std::string_view, N>& strings, const std::string& delimiter = ", ") -> std::string
    {
        return join_to_string(strings, delimiter, std::identity{});
    }

    // FUNCTIONS
    auto to_string(const std::vector<std::string>& strings, const std::string& delimiter = ", ") -> std::string;
    auto to_string(const std::vector<double>& values, const std::string& delimiter = ", ") -> std::string;
    auto to_string_epoch(double epoch_seconds) -> std::string;
} // namespace zenslam::utils

template <typename T>
auto operator+=(std::vector<T>& vec, const std::vector<T>& other) -> std::vector<T>&
{
    vec.insert(vec.end(), other.begin(), other.end());
    return vec;
}
