#include "zenslam/processor.h"

#include <thread>
#include <utility>

#include "zenslam/utils.h"
#include "zenslam/utils_opencv.h"

zenslam::processor::processor(slam_options options, calibration calibration) :
    _options { std::move(options) },
    _calibration { std::move(calibration) },
    _integrator { _calibration.imu, _options.integrator_method }
{
}

auto zenslam::processor::process(const frame::sensor& sensor) -> frame::processed
{
    frame::processed processed = { sensor };

    // Convert to grayscale
    {
        std::jthread thread_0
        {
            [&]()
            {
                processed.images[0] = utils::convert_color(sensor.images[0], cv::COLOR_BGR2GRAY);

                if (_options.clahe_enabled)
                {
                    processed.images[0] = utils::apply_clahe(processed.images[0], _clahe);
                }

                processed.undistorted[0] = utils::rectify(processed.images[0], _calibration.map_x[0], _calibration.map_y[0]);
                processed.pyramids[0]    = utils::pyramid(processed.undistorted[0], _options);
            }
        };

        std::jthread thread_1
        {
            [&]()
            {
                processed.images[1] = utils::convert_color(sensor.images[1], cv::COLOR_BGR2GRAY);

                if (_options.clahe_enabled)
                {
                    processed.images[1] = utils::apply_clahe(processed.images[1], _clahe);
                }

                processed.undistorted[1] = utils::rectify(processed.images[1], _calibration.map_x[1], _calibration.map_y[1]);
                processed.pyramids[1]    = utils::pyramid(processed.undistorted[1], _options);
            }
        };

        std::jthread thread_imu
        {
            [&]
            {
                processed.integral = _integrator.integrate(sensor.imu_data, isnan(_timestamp) ? sensor.timestamp : _timestamp, sensor.timestamp);
            }
        };
    }

    _timestamp = sensor.timestamp;

    return processed;
}
