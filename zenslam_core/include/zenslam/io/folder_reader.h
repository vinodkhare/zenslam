#pragma once

#include <filesystem>
#include <vector>

#include "zenslam/io/folder_options.h"
#include "zenslam/frame/sensor.h"

namespace zenslam
{
    class folder_reader
    {
    public:
        using path_type = std::filesystem::path;

        folder_reader
        (
            const path_type& left_dir,
            const path_type& right_dir,
            double           timescale = 1E-9,
            const path_type& imu_file  = ""
        );

        explicit folder_reader(const class folder_options& options);

        [[nodiscard]] std::size_t size() const noexcept
        {
            return _count;
        }

        [[nodiscard]] bool empty() const noexcept
        {
            return _count == 0;
        }

        // Read next frame and return it
        frame::sensor read();

        // Check if more frames are available
        [[nodiscard]] bool has_more() const noexcept
        {
            return _current_index < _count;
        }

        // Reset to beginning
        void reset() noexcept
        {
            _current_index = 0;
        }

    private:
        void        scan_directories(const path_type& left_dir, const path_type& right_dir);
        void        load_imu_data(const path_type& imu_file);
        static bool is_image_file(const path_type& p);

        std::vector<path_type>  _left_files;
        std::vector<path_type>  _right_files;
        double                  _timescale     = 1E-9;
        std::size_t             _count         = 0;
        std::size_t             _current_index = 0;
        std::vector<frame::imu> _imu_measurements;
        std::size_t             _imu_index = 0;
    };
} // namespace zenslam
