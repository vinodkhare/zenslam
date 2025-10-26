#pragma once

#include <vector>
#include <optional>
#include <opencv2/core/affine.hpp>
#include "imu_calibration.h"
#include "frame/sensor.h"

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
     * @brief IMU pre-integration using UGPM (Unified Gaussian Preintegrated Measurements)
     * 
     * This class encapsulates the IMU pre-integration process with proper overlap handling
     * for optimal UGPM performance. UGPMs need a bit of data overlap between integration 
     * windows (8*state period) to perform optimally.
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
     *     std::cout << "Delta rotation determinant: " << result->delta_R.determinant() << std::endl;
     * }
     * @endcode
     */
    class preint
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
        explicit preint
        (
            const imu_calibration& imu_calib,
            method                 preint_method = method::ugpm
        );

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

#if ZENSLAM_HAS_UGPM
        /**
         * @brief Integrate IMU measurements over a time interval
         * 
         * @param measurements IMU measurements to integrate
         * @param start_time Start of integration interval (seconds)
         * @param end_time End of integration interval (seconds)
         * @return Preintegrated measurement result, or nullopt if integration fails
         */
        auto integrate
        (
            const std::vector<frame::imu_measurement>& measurements,
            double                                     start_time,
            double                                     end_time
        ) -> std::optional<ugpm::PreintMeas>;

        /**
         * @brief Integrate with overlap from previous window
         * 
         * This method automatically includes overlap data from the previous integration
         * window for optimal UGPM performance.
         * 
         * @param measurements Current IMU measurements
         * @param start_time Start of current integration interval
         * @param end_time End of current integration interval
         * @return Preintegrated measurement result, or nullopt if integration fails
         */
        auto integrate_with_overlap
        (
            const std::vector<frame::imu_measurement>& measurements,
            double                                     start_time,
            double                                     end_time
        ) -> std::optional<ugpm::PreintMeas>;

        /**
         * @brief Get identity preintegrated measurement (no motion)
         * 
         * @return PreintMeas with identity rotation, zero velocity/position
         */
        static auto identity() -> ugpm::PreintMeas;

        /**
         * @brief Predict pose from IMU pre-integration
         * 
         * Given the state at time t0 (pose, velocity) and preintegrated IMU measurements,
         * predict the pose at time t1 using:
         *   R_1 = R_0 * delta_R
         *   v_1 = v_0 + R_0 * delta_v + g * dt
         *   p_1 = p_0 + v_0 * dt + R_0 * delta_p + 0.5 * g * dt^2
         * 
         * @param pose_0 Initial pose at time t0
         * @param velocity_0 Initial velocity at time t0 in world frame (m/s)
         * @param preint_meas Preintegrated IMU measurement from t0 to t1
         * @param gravity Gravity vector in world frame (default: [0, 0, -9.81] m/s²)
         * @return Predicted pose at time t1
         */
        static auto predict_pose
        (
            const cv::Affine3d&     pose_0,
            const cv::Vec3d&        velocity_0,
            const ugpm::PreintMeas& preint_meas,
            const cv::Vec3d&        gravity = cv::Vec3d(0, 0, -9.81)
        ) -> cv::Affine3d;

        /**
         * @brief Predict velocity from IMU pre-integration
         * 
         * Given the state at time t0 and preintegrated IMU measurements,
         * predict the velocity at time t1 using:
         *   v_1 = v_0 + R_0 * delta_v + g * dt
         * 
         * @param pose_0 Initial pose at time t0 (for rotation)
         * @param velocity_0 Initial velocity at time t0 in world frame (m/s)
         * @param preint_meas Preintegrated IMU measurement from t0 to t1
         * @param gravity Gravity vector in world frame (default: [0, 0, -9.81] m/s²)
         * @return Predicted velocity at time t1 in world frame (m/s)
         */
        static auto predict_velocity
        (
            const cv::Affine3d&     pose_0,
            const cv::Vec3d&        velocity_0,
            const ugpm::PreintMeas& preint_meas,
            const cv::Vec3d&        gravity = cv::Vec3d(0, 0, -9.81)
        ) -> cv::Vec3d;
#endif

    private:
#if ZENSLAM_HAS_UGPM
        /**
         * @brief Convert frame::imu_measurement to ugpm::ImuData
         */
        auto to_ugpm_data
        (
            const std::vector<frame::imu_measurement>& measurements
        ) -> ugpm::ImuData;

        imu_calibration _imu_calib;
        method          _method;
        int             _overlap_factor;
        double          _state_freq;
        bool            _correlate;

        std::vector<double> _acc_bias;
        std::vector<double> _gyr_bias;

        // Store previous measurements for overlap
        std::vector<frame::imu_measurement> _prev_measurements;
        double                              _prev_end_time;
#else
        // Dummy members when ugpm is not available
        int _dummy;
#endif
    };
} // namespace zenslam
