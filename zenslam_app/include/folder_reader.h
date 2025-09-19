#pragma once

#include <filesystem>
#include <iterator>
#include <vector>
#include <opencv2/core.hpp>

namespace zenslam
{
    // FolderReader: collects image file paths and offers random-access iteration.
    // cv::Mat uses reference counting, so returning by value is inexpensive.
    class folder_reader
    {
    public:
        using path_type = std::filesystem::path;

        explicit folder_reader(const path_type &directory, bool recursive = false);

        [[nodiscard]] std::size_t                   size() const noexcept { return _files.size(); }
        [[nodiscard]] bool                          empty() const noexcept { return _files.empty(); }
        [[nodiscard]] const std::vector<path_type> &paths() const noexcept { return _files; }

        // Load image at index (lazy). Returns empty Mat if load fails.
        cv::Mat operator[](std::size_t idx) const;

        struct iterator
        {
            using iterator_category = std::random_access_iterator_tag;
            using value_type        = cv::Mat;
            using difference_type   = std::ptrdiff_t;
            using pointer           = void;    // not providing pointer semantics
            using reference         = cv::Mat; // cv::Mat returned by value

            iterator() = default;

            iterator(const folder_reader *owner, const std::size_t index) : owner_(owner), index_(index)
            {
            }

            reference operator*() const { return (*owner_)[index_]; }
            reference operator[](const difference_type n) const { return (*owner_)[index_ + n]; }

            iterator &operator++()
            {
                ++index_;
                return *this;
            }

            iterator operator++(int)
            {
                const auto tmp = *this;
                ++*this;
                return tmp;
            }

            iterator &operator--()
            {
                --index_;
                return *this;
            }

            iterator operator--(int)
            {
                const auto tmp = *this;
                --*this;
                return tmp;
            }

            iterator &operator+=(difference_type n)
            {
                index_ += n;
                return *this;
            }

            iterator &operator-=(difference_type n)
            {
                index_ -= n;
                return *this;
            }

            friend iterator operator+(iterator it, difference_type n)
            {
                it += n;
                return it;
            }

            friend iterator operator+(difference_type n, iterator it)
            {
                it += n;
                return it;
            }

            friend iterator operator-(iterator it, difference_type n)
            {
                it -= n;
                return it;
            }

            friend difference_type operator-(const iterator &a, const iterator &b)
            {
                return static_cast<difference_type>(a.index_) - static_cast<difference_type>(b.index_);
            }

            friend bool operator==(const iterator &a, const iterator &b)
            {
                return a.owner_ == b.owner_ && a.index_ == b.index_;
            }

            friend bool operator!=(const iterator &a, const iterator &b) { return !(a == b); }
            friend bool operator<(const iterator &a, const iterator &b) { return a.index_ < b.index_; }
            friend bool operator>(const iterator &a, const iterator &b) { return b < a; }
            friend bool operator<=(const iterator &a, const iterator &b) { return b >= a; }
            friend bool operator>=(const iterator &a, const iterator &b) { return !(a < b); }

        private:
            const folder_reader *owner_ = nullptr;
            std::size_t          index_ = 0;
        };

        [[nodiscard]] iterator begin() const { return {this, 0}; }
        [[nodiscard]] iterator end() const { return {this, _files.size()}; }

    private:
        void scan(const path_type &directory, bool recursive);

        static bool is_image_file(const path_type &p);

        std::vector<path_type> _files;
    };
} // namespace zenslam
