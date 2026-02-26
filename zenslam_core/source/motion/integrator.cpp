#include "zenslam/motion/integrator.h"

#include <utility>
#include <preint/preint.h>
#include <basalt/imu/preintegration.h>
#include <spdlog/spdlog.h>

namespace zenslam
{
    integrator::integrator(imu_calibration imu_calib, const method preint_method)
        : _imu_calib(std::move(imu_calib)),
          _method(preint_method)
    {
    }

    static ugpm::PreintPrior to_ugpm_prior(const prior& p)
    {
        ugpm::PreintPrior up;
        up.acc_bias = p.acc_bias;
        up.gyr_bias = p.gyr_bias;
        return up;
    }

    static prior from_ugpm_prior(const ugpm::PreintPrior& p)
    {
        prior zp;
        zp.acc_bias = p.acc_bias;
        zp.gyr_bias = p.gyr_bias;
        return zp;
    }

    static cv::Matx33d eigenToMatx33d(const Eigen::Matrix3d& M)
    {
        return {
            M(0, 0),
            M(0, 1),
            M(0, 2),
            M(1, 0),
            M(1, 1),
            M(1, 2),
            M(2, 0),
            M(2, 1),
            M(2, 2)
        };
    }

    static cv::Vec3d eigenToVec3d(const Eigen::Vector3d& v)
    {
        return { v(0), v(1), v(2) };
    }

    static cv::Matx<double, 9, 9> eigenToMatx9x9(const Eigen::Matrix<double, 9, 9>& M)
    {
        cv::Matx<double, 9, 9> out;
        for (auto r = 0; r < 9; ++r)
            for (auto c = 0; c < 9; ++c)
                out(r, c) = M(r, c);
        return out;
    }

    static integral from_ugpm_meas(const ugpm::PreintMeas& m)
    {
        integral z { };
        z.delta_R    = eigenToMatx33d(m.delta_R);
        z.delta_v    = eigenToVec3d(m.delta_v);
        z.delta_p    = eigenToVec3d(m.delta_p);
        z.dt         = m.dt;
        z.dt_sq_half = m.dt_sq_half;
        z.cov        = eigenToMatx9x9(m.cov);
        return z;
    }

    static integral integrate_with_basalt
    (
        const imu_calibration&         imu_calib,
        const prior&                   prior_biases,
        const std::vector<frame::imu>& measurements,
        const double                   start,
        const double                   end
    )
    {
        // Configure Basalt preintegration - noise variances as Vec3
        basalt::IntegratedImuMeasurement<double>::Vec3 accel_cov_vec;
        accel_cov_vec <<
            imu_calib.accelerometer_noise_density * imu_calib.accelerometer_noise_density,
            imu_calib.accelerometer_noise_density * imu_calib.accelerometer_noise_density,
            imu_calib.accelerometer_noise_density * imu_calib.accelerometer_noise_density;

        basalt::IntegratedImuMeasurement<double>::Vec3 gyro_cov_vec;
        gyro_cov_vec <<
            imu_calib.gyroscope_noise_density * imu_calib.gyroscope_noise_density,
            imu_calib.gyroscope_noise_density * imu_calib.gyroscope_noise_density,
            imu_calib.gyroscope_noise_density * imu_calib.gyroscope_noise_density;

        basalt::IntegratedImuMeasurement<double>::Vec3 accel_bias_vec;
        accel_bias_vec << prior_biases.acc_bias[0], prior_biases.acc_bias[1], prior_biases.acc_bias[2];

        basalt::IntegratedImuMeasurement<double>::Vec3 gyro_bias_vec;
        gyro_bias_vec << prior_biases.gyr_bias[0], prior_biases.gyr_bias[1], prior_biases.gyr_bias[2];

        // Create Basalt preintegration object (time in nanoseconds)
        const auto start_ns = static_cast<int64_t>(start * 1e9);

        basalt::IntegratedImuMeasurement preint_meas { start_ns, gyro_bias_vec, accel_bias_vec };

        // Integrate IMU measurements
        size_t integrated_count = 0;
        for (const auto& [timestamp, acc, gyr] : measurements)
        {
            if (timestamp <= start) continue;
            if (timestamp > end) break;

            basalt::ImuData<double> imu_data;
            imu_data.t_ns = static_cast<int64_t>(timestamp * 1e9);
            imu_data.accel << acc[0], acc[1], acc[2];
            imu_data.gyro << gyr[0], gyr[1], gyr[2];

            preint_meas.integrate(imu_data, accel_cov_vec, gyro_cov_vec);
            integrated_count++;
        }

        if (integrated_count == 0)
        {
            SPDLOG_WARN("Basalt: No IMU samples found in interval [{:.4f}, {:.4f}]", start, end);
            return integral { };
        }

        // Convert Basalt result to zenslam format
        const auto& delta_state = preint_meas.getDeltaState();
        const auto& cov         = preint_meas.get_cov();

        integral result;
        result.delta_R    = eigenToMatx33d(delta_state.T_w_i.so3().matrix());
        result.delta_v    = eigenToVec3d(delta_state.vel_w_i);
        result.delta_p    = eigenToVec3d(delta_state.T_w_i.translation());
        result.dt         = end - start;
        result.dt_sq_half = result.dt * result.dt * 0.5;
        result.cov        = eigenToMatx9x9(cov);

        SPDLOG_DEBUG("Basalt: Integrated {} IMU samples over {:.4f}s", integrated_count, result.dt);

        return result;
    }

    auto integrator::integrate(const std::vector<frame::imu>& measurements, const double start, const double end) -> integral
    {
        // Add all new measurements to _measurements
        for (const auto& m : measurements)
        {
            _measurements.emplace_back(m);
        }

        if (_measurements.empty() || start >= end)
        {
            SPDLOG_WARN("No IMU measurements to integrate or invalid time interval [{:.4f}, {:.4f}]", start, end);
            return integral { };
        }

        // Use Basalt method if selected
        if (_method == method::basalt)
        {
            auto result = integrate_with_basalt(_imu_calib, _prior, _measurements, start, end);

            // Clean up old measurements (keep some buffer for overlap if needed)
            constexpr auto buffer_window = 0.5; // Keep 0.5s of history
            erase_if(_measurements, [&](auto& m) { return m.timestamp < start - buffer_window; });

            return result;
        }

        // UGPM/LPM path
        constexpr auto overlap_factor = 8;
        const auto     overlap_period = (end - start) * overlap_factor; // 8 times of period as advised

        std::vector<frame::imu> to_integrate = { };
        for (const auto& m : _measurements)
        {
            if (start - overlap_period <= m.timestamp && m.timestamp <= end + overlap_period)
            {
                to_integrate.push_back(m);
            }
        }

        erase_if(_measurements, [&](auto& m) { return m.timestamp < start - overlap_period; });

        ugpm::ImuData imu_data = { };
        imu_data.acc.reserve(to_integrate.size());
        imu_data.gyr.reserve(to_integrate.size());

        for (const auto& [timestamp, acc, gyr] : to_integrate)
        {
            imu_data.acc.emplace_back(ugpm::ImuSample { timestamp, acc[0], acc[1], acc[2] });
            imu_data.gyr.emplace_back(ugpm::ImuSample { timestamp, gyr[0], gyr[1], gyr[2] });
        }

        // Setup preintegration options
        ugpm::PreintOption options = { };
        options.type               = _method == method::ugpm ? ugpm::UGPM : ugpm::LPM;
        options.state_freq         = 200;
        options.correlate          = true;
        options.quantum            = -1; // Disable per-chunk mode

        ugpm::ImuPreintegration integration = { imu_data, start, end, options, to_ugpm_prior(_prior), false, overlap_factor };

        _prior = from_ugpm_prior(integration.getPrior());

        return from_ugpm_meas(integration.get());
    }
} // namespace zenslam
