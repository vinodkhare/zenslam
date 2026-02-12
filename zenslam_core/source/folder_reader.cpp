#include "zenslam/folder_reader.h"

#include <algorithm>
#include <opencv2/imgcodecs.hpp>
#include <rapidcsv.h>
#include <spdlog/spdlog.h>

namespace zenslam
{
    namespace
    {
        bool is_equals(const std::string& a, const std::string& b)
        {
            if (a.size() != b.size())
                return false;
            for (size_t i = 0; i < a.size(); ++i)
                if (std::tolower(static_cast<unsigned char>(a[i])) != std::tolower(static_cast<unsigned char>(b[i])))
                    return false;
            return true;
        }
    } // namespace

    bool folder_reader::is_image_file(const path_type& p)
    {
        static const char* extensions[] = { ".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff" };

        const auto ext = p.extension().string();

        return std::ranges::any_of(extensions, [&ext](auto e) { return is_equals(e, ext); });
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
                if (entry.is_regular_file() && is_image_file(entry.path()))
                {
                    _left_files.push_back(entry.path());
                }
            }
            std::ranges::sort(_left_files);
        }
        else
        {
            SPDLOG_ERROR("left camera folder does not exist: {}", left_dir.string());
        }

        // Scan right directory
        if (std::filesystem::exists(right_dir) && std::filesystem::is_directory(right_dir))
        {
            for (auto& entry : std::filesystem::directory_iterator(right_dir))
            {
                if (entry.is_regular_file() && is_image_file(entry.path()))
                {
                    _right_files.push_back(entry.path());
                }
            }
            std::ranges::sort(_right_files);
        }

        _count = std::min(_left_files.size(), _right_files.size());
    }

    folder_reader::folder_reader(const path_type& left_dir, const path_type& right_dir, const double timescale, const path_type& imu_file) :
        _timescale(timescale)
    {
        scan_directories(left_dir, right_dir);
        if (!imu_file.empty())
        {
            load_imu_data(imu_file);
        }
    }

    folder_reader::folder_reader(const class folder_options& options) : _timescale(options.timescale)
    {
        scan_directories(options.root / options.left, options.root / options.right);
        if (!options.imu_file.value().empty())
        {
            load_imu_data(options.imu_file);
        }
    }

    void folder_reader::load_imu_data(const path_type& imu_file)
    {
        if (!std::filesystem::exists(imu_file))
        {
            SPDLOG_WARN("IMU file not found: {}", imu_file.string());
            return;
        }

        try
        {
            // Read CSV file with rapidcsv
            // TUM-VI format: timestamp[ns],omega_x[rad/s],omega_y,omega_z,alpha_x[m/s^2],alpha_y,alpha_z
            const rapidcsv::Document doc(imu_file.string(), rapidcsv::LabelParams(0, -1));

            const size_t row_count = doc.GetRowCount();
            _imu_measurements.reserve(row_count);

            for (size_t i = 0; i < row_count; ++i)
            {
                frame::imu imu;

                // Column 0: timestamp in nanoseconds
                const auto timestamp_ns = doc.GetCell<double>(0, i);
                imu.timestamp           = timestamp_ns * _timescale;

                // Columns 1-3: angular velocity (rad/s)
                imu.gyr = cv::Vec3d(doc.GetCell<double>(1, i), doc.GetCell<double>(2, i), doc.GetCell<double>(3, i));

                imu.acc = cv::Vec3d(doc.GetCell<double>(4, i), doc.GetCell<double>(5, i), doc.GetCell<double>(6, i));

                _imu_measurements.push_back(imu);
            }

            SPDLOG_INFO("Loaded {} IMU measurements from {}", _imu_measurements.size(), imu_file.string());
        }
        catch (const std::exception& e)
        {
            SPDLOG_ERROR("Failed to load IMU data from {}: {}", imu_file.string(), e.what());
        }
    }

    frame::sensor folder_reader::read()
    {
        if (_current_index >= _count)
        {
            return {}; // Return empty frame if no more data
        }

        frame::sensor frame = {};

        // Extract timestamp from filename (assumed to be in nanoseconds)
        double timestamp_ns;
        try
        {
            timestamp_ns = std::stod(_left_files[_current_index].stem().string());
        }
        catch (const std::invalid_argument& e)
        {
            SPDLOG_ERROR("Invalid timestamp in filename: {}", _left_files[_current_index].stem().string());
            throw;
        }

        frame.timestamp = timestamp_ns * _timescale;

        // Load images
        frame.images[0] = cv::imread(_left_files[_current_index].string());
        frame.images[1] = cv::imread(_right_files[_current_index].string());

        // if images are empty, throw error
        if (frame.images[0].empty())
        {
            throw std::runtime_error("Failed to load left image: " + _left_files[_current_index].string());
        }

        if (frame.images[1].empty())
        {
            throw std::runtime_error("Failed to load right image: " + _right_files[_current_index].string());
        }

        // Set frame index
        frame.index = frame::sensor::count++;

        // Extract IMU measurements between previous frame and current frame
        if (!_imu_measurements.empty())
        {
            // Get previous frame timestamp (or 0 for first frame)
            const double prev_timestamp = (_current_index > 0) ? std::stod(_left_files[_current_index - 1].stem().string()) * _timescale : 0.0;

            // Find all IMU measurements in the interval (prev_timestamp, frame.timestamp]
            while (_imu_index < _imu_measurements.size() && _imu_measurements[_imu_index].timestamp <= prev_timestamp)
            {
                ++_imu_index;
            }

            // Collect IMU measurements in the current interval
            while (_imu_index < _imu_measurements.size() && _imu_measurements[_imu_index].timestamp <= frame.timestamp)
            {
                frame.imu_data.push_back(_imu_measurements[_imu_index]);
                ++_imu_index;
            }

            if (_current_index == 0 && !frame.imu_data.empty())
            {
                SPDLOG_DEBUG("First frame has {} IMU measurements", frame.imu_data.size());
            }
        }

        // Advance to next frame
        ++_current_index;

        SPDLOG_TRACE("Read image index {} with timestamp {:.6f} s", frame.index, frame.timestamp);

        return frame;
    }
} // namespace zenslam
