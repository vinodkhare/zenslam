#include "zenslam/integrator.h"

#include <utility>
#include <spdlog/spdlog.h>

namespace zenslam
{
    integrator::integrator(imu_calibration imu_calib, method preint_method)
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

    static integral from_ugpm_meas(const ugpm::PreintMeas& m)
    {
        integral z { };
        z.delta_R    = m.delta_R;
        z.delta_v    = m.delta_v;
        z.delta_p    = m.delta_p;
        z.dt         = m.dt;
        z.dt_sq_half = m.dt_sq_half;
        z.cov        = m.cov;
        return z;
    }

    auto integrator::integrate(const std::vector<frame::imu>& measurements, const double start, const double end) -> integral
    {
        // first add all new measurements to _measurements
        for (const auto& m : measurements)
        {
            _measurements.emplace_back(m);
        }

        if (_measurements.empty() || start >= end)
        {
            SPDLOG_WARN("No IMU measurements to integrate or invalid time interval [{:.4f}, {:.4f}]", start, end);
            return integral { };
        }

        constexpr int overlap_factor = 8;
        const auto    overlap_period = (end - start) * overlap_factor; // 8 times of period as advised

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
