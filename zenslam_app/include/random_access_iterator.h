#pragma once
#include <iterator>
#include <cstddef>

namespace zenslam {

template <typename Owner, typename ValueType>
class random_access_iterator {
public:
    using iterator_category = std::random_access_iterator_tag;
    using value_type        = ValueType;
    using difference_type   = std::ptrdiff_t;
    using pointer           = void;
    using reference         = ValueType;

    random_access_iterator() = default;
    random_access_iterator(const Owner* owner, std::size_t index)
        : owner_(owner), index_(index) {}

    reference operator*() const { return (*owner_)[index_]; }
    reference operator[](difference_type n) const { return (*owner_)[index_ + n]; }

    random_access_iterator& operator++() { ++index_; return *this; }
    random_access_iterator operator++(int) { auto tmp = *this; ++*this; return tmp; }
    random_access_iterator& operator--() { --index_; return *this; }
    random_access_iterator operator--(int) { auto tmp = *this; --*this; return tmp; }
    random_access_iterator& operator+=(difference_type n) { index_ += n; return *this; }
    random_access_iterator& operator-=(difference_type n) { index_ -= n; return *this; }
    friend random_access_iterator operator+(random_access_iterator it, difference_type n) { it += n; return it; }
    friend random_access_iterator operator+(difference_type n, random_access_iterator it) { it += n; return it; }
    friend random_access_iterator operator-(random_access_iterator it, difference_type n) { it -= n; return it; }
    friend difference_type operator-(const random_access_iterator& a, const random_access_iterator& b) { return static_cast<difference_type>(a.index_) - static_cast<difference_type>(b.index_); }
    friend bool operator==(const random_access_iterator& a, const random_access_iterator& b) { return a.owner_ == b.owner_ && a.index_ == b.index_; }
    friend bool operator!=(const random_access_iterator& a, const random_access_iterator& b) { return !(a == b); }
    friend bool operator<(const random_access_iterator& a, const random_access_iterator& b) { return a.index_ < b.index_; }
    friend bool operator>(const random_access_iterator& a, const random_access_iterator& b) { return b < a; }
    friend bool operator<=(const random_access_iterator& a, const random_access_iterator& b) { return !(b < a); }
    friend bool operator>=(const random_access_iterator& a, const random_access_iterator& b) { return !(a < b); }
private:
    const Owner* owner_ = nullptr;
    std::size_t index_ = 0;
};

} // namespace zenslam
