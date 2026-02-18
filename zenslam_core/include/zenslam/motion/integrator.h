#pragma once

#include <vector>

#include "zenslam/calibration/imu_calibration.h"
#include "zenslam/frame/imu.h"
#include "zenslam/motion/integrator_types.h"

namespace zenslam
{
    /**
     * @brief IMU pre-integration using UGPM (Unified Gaussian Preintegrated
     * Measurements)
     *
     * This class encapsulates the IMU pre-integration process with proper overlap
     * handling for optimal UGPM performance. UGPMs need a bit of data overlap
     * between integration windows (8*state period) to perform optimally.
     *
     * Example usage:
     * @code
     * preint imu_preint(calibration.imu);
     * imu_preint.set_overlap_factor(8);  // 8x state period overlap (default)
     *
     * auto result = imu_preint.integrate(
     *     sensor.imu_data,
     *     start_timestamp,
     *     end_timestamp
     * );
     *
     * if (result) {
     *     std::cout << "Delta rotation determinant: " <<
     * result->delta_R.determinant() << std::endl;
     * }
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
            ugpm, ///< Unified Gaussian Preintegrated Measurements (accurate, requires optimization)
            lpm   ///< Linear Preintegrated Measurements (fast, no optimization)
        };

        /**
         * @brief Construct pre-integrator with IMU calibration
         *
         * @param imu_calib IMU calibration parameters containing noise densities
         * @param preint_method Integration method (default: UGPM)
         */
        explicit integrator(imu_calibration imu_calib, method preint_method = method::ugpm);

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
