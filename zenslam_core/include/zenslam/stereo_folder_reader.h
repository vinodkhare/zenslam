#pragma once

#include <algorithm>

#include "mono_folder_reader.h"
#include "options.h"
#include "random_access_iterator.h"
#include "stereo_frame.h"

namespace zenslam
{
    class stereo_folder_reader
    {
    public:
        using iterator  = random_access_iterator<stereo_folder_reader, stereo_frame>;
        using path_type = std::filesystem::path;

        stereo_folder_reader
        (
            const path_type &left_dir,
            const path_type &right_dir,
            const double     timescale = 1E-9
        );

        explicit stereo_folder_reader(const class options::folder &options);

        [[nodiscard]] std::size_t size() const noexcept
        {
            return std::min(_left.size(), _right.size());
        }

        [[nodiscard]] bool empty() const noexcept
        {
            return size() == 0;
        }

        stereo_frame operator[](const std::size_t idx) const
        {
            return stereo_frame { _left[idx], _right[idx] };
        }

        [[nodiscard]] iterator begin() const
        {
            return iterator { this, 0 };
        }

        [[nodiscard]] iterator end() const
        {
            return iterator { this, size() };
        }

        [[nodiscard]] const mono_folder_reader &left() const noexcept
        {
            return _left;
        }

        [[nodiscard]] const mono_folder_reader &right() const noexcept
        {
            return _right;
        }

    private:
        mono_folder_reader _left;
        mono_folder_reader _right;
    };
} // namespace zenslam