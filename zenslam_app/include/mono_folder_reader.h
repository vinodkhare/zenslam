#pragma once

#include <filesystem>
#include <vector>
#include <opencv2/core.hpp>

#include "mono_frame.h"
#include "random_access_iterator.h"

namespace zenslam
{
    // mono_folder_reader: collects image file paths and offers random-access iteration.
    // The element type is zenslam::mono_frame (contains timestamp and cv::Mat image).
    class mono_folder_reader
    {
    public:
        using path_type = std::filesystem::path;

        explicit mono_folder_reader(const path_type &directory, bool recursive = false, double timescale = 1E-9);

        [[nodiscard]] std::size_t                   size() const noexcept { return _files.size(); }
        [[nodiscard]] bool                          empty() const noexcept { return _files.empty(); }
        [[nodiscard]] const std::vector<path_type> &paths() const noexcept { return _files; }

        // Load mono_frame at index (lazy). Returns mono_frame with empty image if load fails.
        mono_frame operator[](std::size_t index) const;

        using iterator = random_access_iterator<mono_folder_reader, mono_frame>;
        [[nodiscard]] iterator begin() const { return iterator(this, 0); }
        [[nodiscard]] iterator end() const { return iterator(this, _files.size()); }

    private:
        void scan(const path_type &directory, bool recursive);

        static bool is_image_file(const path_type &p);

        std::vector<path_type> _files     = {};
        double                 _timescale = 1E-9;
    };
} // namespace zenslam
