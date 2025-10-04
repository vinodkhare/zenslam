#include "mono_folder_reader.h"

#include <algorithm>

#include <opencv2/imgcodecs.hpp>

#include <spdlog/spdlog.h>

namespace zenslam { namespace
    {
        bool is_equals(const std::string &a, const std::string &b)
        {
            if (a.size() != b.size())
                return false;
            for (size_t i = 0; i < a.size(); ++i)
                if (std::tolower(static_cast<unsigned char>(a[i])) !=
                    std::tolower(static_cast<unsigned char>(b[i])))
                    return false;
            return true;
        }
    } // namespace

    bool mono_folder_reader::is_image_file(const path_type &p)
    {
        static const char *extensions[] = {
            ".png", ".jpg", ".jpeg",
            ".bmp", ".tif", ".tiff"
        };

        const auto ext = p.extension().string();

        return std::ranges::any_of(extensions, [&ext](auto e) { return is_equals(e, ext); });
    }

    void mono_folder_reader::scan(const path_type &directory, const bool recursive)
    {
        _files.clear();

        if (!std::filesystem::exists(directory) ||
            !std::filesystem::is_directory(directory))
        {
            return;
        }

        if (recursive)
        {
            for (auto &entry:
                 std::filesystem::recursive_directory_iterator(directory))
            {
                if (entry.is_regular_file() && is_image_file(entry.path()))
                {
                    _files.push_back(entry.path());
                }
            }
        } else
        {
            for (auto &entry: std::filesystem::directory_iterator(directory))
            {
                if (entry.is_regular_file())
                {
                    _files.push_back(entry.path());
                }
            }
        }

        std::ranges::sort(_files);
    }

    mono_folder_reader::mono_folder_reader(const path_type &directory, const bool recursive, const double timescale)
    {
        scan(directory, recursive);

        _timescale = timescale;
    }

    mono_frame mono_folder_reader::operator[](const std::size_t index) const
    {
        if (index >= _files.size())
        {
            return { }; // return empty Mat (or throw)
        }

        // treat filename as time-stamp in nanoseconds
        return
        {
            (std::stod(_files[index].stem().string()) * _timescale),
            cv::imread(_files[index].string())
        };
    }
} // namespace zenslam
