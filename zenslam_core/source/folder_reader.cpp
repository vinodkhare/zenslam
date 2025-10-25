#include "folder_reader.h"

#include <algorithm>
#include <opencv2/imgcodecs.hpp>

namespace zenslam
{
    namespace
    {
        bool is_equals(const std::string& a, const std::string& b)
        {
            if (a.size() != b.size()) return false;
            for (size_t i = 0; i < a.size(); ++i)
                if (std::tolower(static_cast<unsigned char>(a[i])) !=
                    std::tolower(static_cast<unsigned char>(b[i])))
                    return false;
            return true;
        }
    }

    bool folder_reader::is_image_file(const path_type& p)
    {
        static const char* extensions[] = {
            ".png", ".jpg", ".jpeg",
            ".bmp", ".tif", ".tiff"
        };

        const auto ext = p.extension().string();

        return std::ranges::any_of
        (
            extensions,
            [&ext](auto e)
            {
                return is_equals(e, ext);
            }
        );
    }

    void folder_reader::scan_directories(const path_type& left_dir, const path_type& right_dir)
    {
        _left_files.clear();
        _right_files.clear();

        // Scan left directory
        if (std::filesystem::exists(left_dir) && std::filesystem::is_directory(left_dir))
        {
            for (auto& entry : std::filesystem::directory_iterator(left_dir))
            {
                if (entry.is_regular_file())
                {
                    _left_files.push_back(entry.path());
                }
            }
            std::ranges::sort(_left_files);
        }

        // Scan right directory
        if (std::filesystem::exists(right_dir) && std::filesystem::is_directory(right_dir))
        {
            for (auto& entry : std::filesystem::directory_iterator(right_dir))
            {
                if (entry.is_regular_file())
                {
                    _right_files.push_back(entry.path());
                }
            }
            std::ranges::sort(_right_files);
        }

        _count = std::min(_left_files.size(), _right_files.size());
    }

    folder_reader::folder_reader
    (
        const path_type& left_dir,
        const path_type& right_dir,
        const double     timescale
    ) :
        _timescale(timescale)
    {
        scan_directories(left_dir, right_dir);
    }

    folder_reader::folder_reader(const class options::folder& options) :
        _timescale(options.timescale)
    {
        scan_directories(options.root / options.left, options.root / options.right);
    }

    frame::sensor folder_reader::read()
    {
        if (_current_index >= _count)
        {
            return {}; // Return empty frame if no more data
        }

        frame::sensor frame = {};

        // Extract timestamp from filename (assumed to be in nanoseconds)
        const auto timestamp_ns = std::stod(_left_files[_current_index].stem().string());
        frame.timestamp = timestamp_ns * _timescale;

        // Load images
        frame.images[0] = cv::imread(_left_files[_current_index].string());
        frame.images[1] = cv::imread(_right_files[_current_index].string());

        // Set frame index
        frame.index = frame::sensor::count++;

        // Advance to next frame
        ++_current_index;

        return frame;
    }
} // namespace zenslam