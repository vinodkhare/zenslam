#pragma once

#include "imu_calibration.h"
#include "zenslam/frame/imu.h"
#include <opencv2/core/affine.hpp>
#include <vector>

// Conditional ugpm support
#if __has_include(<preint/preint.h>)
#define ZENSLAM_HAS_UGPM 1
#include <preint/preint.h>
#else
#define ZENSLAM_HAS_UGPM 0
#endif

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
            ugpm, ///< Unified Gaussian Preintegrated Measurements (accurate, requires
          ///< optimization)
            lpm   ///< Linear Preintegrated Measurements (fast, no optimization)
        };

        /**
   * @brief Construct pre-integrator with IMU calibration
   *
   * @param imu_calib IMU calibration parameters containing noise densities
   * @param preint_method Integration method (default: UGPM)
   */
        explicit integrator(imu_calibration imu_calib, method preint_method = method::ugpm);

        /**
           * @brief Set the overlap factor for UGPM optimization
           *
           * UGPMs need data overlap between integration windows to perform optimally.
           * The overlap duration is computed as: overlap_factor * (1.0 / state_freq)
           *
           * @param factor Overlap factor (default: 8, recommended range: 4-12)
           */
        void set_overlap_factor(int factor);

        /**
   * @brief Set the state frequency for UGPM
   *
   * @param freq State frequency in Hz (default: 50.0)
   */
        void set_state_frequency(double freq);

        /**
   * @brief Enable/disable covariance correlation in UGPM
   *
   * @param enable True to enable correlation (more accurate but slower)
   */
        void set_correlate(bool enable);

        /**
         * @brief Set prior IMU biases
         *
         * @param acc_bias Accelerometer bias [x, y, z] in m/s²
         * @param gyr_bias Gyroscope bias [x, y, z] in rad/s
         */
        void set_biases
        (
            const std::vector<double>& acc_bias,
            const std::vector<double>& gyr_bias
        );

        /** Integrate IMU measurements over a time interval with proper overlap handling.
         *
         * @param measurements IMU measurements to integrate
         * @param start start time of integration interval
         * @param end end time of integration interval
         * @return pre-integrated IMU measurements over [start, end]
         */
        auto integrate(const std::vector<frame::imu>& measurements, double start, double end) -> ugpm::PreintMeas;

    private:
        imu_calibration _imu_calib;
        method          _method;
        int             _overlap_factor;
        double          _state_freq;
        bool            _correlate;

        std::vector<double> _acc_bias;
        std::vector<double> _gyr_bias;

        // Store previous measurements for overlap
        std::vector<frame::imu> _measurements;
        double                  _prev_end_time;
        double                  _overlap = 0.0; // overlap interval in seconds
        ugpm::PreintPrior       _prior   = { };
    };
} // namespace zenslam
