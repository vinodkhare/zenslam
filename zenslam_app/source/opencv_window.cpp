#include "opencv_window.h"
#include <opencv2/highgui.hpp>
#include <zenslam/utils/utils_opencv.h>

namespace zenslam
{
    opencv_window::opencv_window(type window_type, const options& options) :
        _type(window_type),
        _options(options)
    {
        switch (_type)
        {
        case type::spatial_matches:
            _window_name = "matches_spatial";
            _window_title = "matches spatial";
            break;
        case type::temporal_matches:
            _window_name = "matches_temporal";
            _window_title = "matches temporal";
            break;
        }
    }

    void opencv_window::initialize()
    {
        if (_initialized)
            return;

        cv::namedWindow(_window_name);
        cv::resizeWindow(_window_name, _window_width, _window_height);
        cv::setWindowTitle(_window_name, _window_title);

        _initialized = true;
    }

    void opencv_window::render(const frame::system& system)
    {
        // ReSharper disable once CppDFAConstantConditions
        if (!_visible)
            // ReSharper disable once CppDFAUnreachableCode
            return;

        if (!_initialized)
            initialize();

        // Check if we have renderable data
        if (system[0].undistorted[0].empty() || system[1].undistorted[0].empty() || system[1].keypoints[0].empty())
            return;

        cv::Mat image;

        switch (_type)
        {
        case type::spatial_matches:
            image = utils::draw_matches_spatial(system[1], system.points3d);
            break;

        case type::temporal_matches:
            image = utils::draw_matches_temporal(system[0], system[1], _options.slam);
            break;
        }

        if (!image.empty())
        {
            cv::imshow(_window_name, image);
        }
    }
} // namespace zenslam
