#pragma once

#include <vector>

#include "zenslam/calibration/imu_calibration.h"
#include "zenslam/frame/imu.h"
#include "zenslam/motion/integrator_types.h"

namespace zenslam
{
    /**
     * @brief IMU pre-integration with multiple backend options
     *
     * This class encapsulates the IMU pre-integration process supporting:
     * - UGPM: Gaussian Process-based (accurate but numerically sensitive)
     * - LPM: Linear Piecewise Model (fast, standard)
     * - Basalt: Manifold-based (numerically stable, modern)
     *
     * Example usage:
     * @code
     * integrator imu_integrator(calibration.imu, integrator::method::basalt);
     *
     * auto result = imu_integrator.integrate(
     *     sensor.imu_data,
     *     start_timestamp,
     *     end_timestamp
     * );
     *
     * std::cout << "Delta rotation determinant: " <<
     *     result.delta_R.determinant() << std::endl;
     * @endcode
     */
    class integrator
    {
    public:
        /**
         * @brief Integration method selection
         */
        enum class method
        {
            ugpm,  ///< Unified Gaussian Preintegrated Measurements (accurate, requires optimization)
            lpm,   ///< Linear Preintegrated Measurements (fast, no optimization)
            basalt ///< Basalt manifold-based preintegration (numerically stable)
        };

        /**
         * @brief Construct pre-integrator with IMU calibration
         *
         * @param imu_calib IMU calibration parameters containing noise densities
         * @param preint_method Integration method (default: Basalt for stability)
         */
        explicit integrator(imu_calibration imu_calib, method preint_method = method::basalt);

        /** Integrate IMU measurements over a time interval with proper overlap handling.
         *
         * @param measurements IMU measurements to integrate
         * @param start start time of integration interval
         * @param end end time of integration interval
         * @return pre-integrated IMU measurements over [start, end] in zenslam format
         */
        auto integrate(const std::vector<frame::imu>& measurements, double start, double end) -> integral;

    private:
        imu_calibration         _imu_calib;
        method                  _method;
        std::vector<frame::imu> _measurements;
        prior                   _prior = { };
    };
} // namespace zenslam
