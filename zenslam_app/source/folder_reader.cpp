#include "folder_reader.h"

#include <algorithm>
#include <opencv2/imgcodecs.hpp>

namespace zenslam {namespace
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

    bool folder_reader::is_image_file(const path_type &p)
    {
        static const char *extensions[] = {
            ".png", ".jpg", ".jpeg",
            ".bmp", ".tif", ".tiff"
        };
        const auto ext = p.extension().string();

        for (auto e: extensions)
        {
            if (is_equals(ext, e))
                return true;
        }
        return false;
    }

    void folder_reader::scan(const path_type &directory, bool recursive)
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

    folder_reader::folder_reader(const path_type &directory, bool recursive)
    {
        scan(directory, recursive);
    }

    cv::Mat folder_reader::operator[](const std::size_t index) const
    {
        if (index >= _files.size())
        {
            return {}; // return empty Mat (or throw)
        }

        return cv::imread(_files[index].string(), cv::IMREAD_UNCHANGED);
    }
} // namespace zenslam
