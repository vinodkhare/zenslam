#include "zenslam/integrator.h"

#include <opencv2/core/affine.hpp>
#include <utility>
#include <spdlog/spdlog.h>

namespace zenslam
{
#if ZENSLAM_HAS_UGPM

    integrator::integrator(imu_calibration imu_calib, method preint_method)
        : _imu_calib(std::move(imu_calib)),
          _method(preint_method),
          _overlap_factor(8) // Default: 8x state period for optimal UGPM
          ,
          _state_freq(50.0) // Default: 50 Hz state frequency
          ,
          _correlate(true) // Default: enable correlation
          ,
          _acc_bias({ 0.0, 0.0, 0.0 }),
          _gyr_bias({ 0.0, 0.0, 0.0 }),
          _prev_end_time(0.0)
    {
    }

    void integrator::set_overlap_factor(const int factor)
    {
        _overlap_factor = factor;
    }

    void integrator::set_state_frequency(const double freq)
    {
        _state_freq = freq;
    }

    void integrator::set_correlate(const bool enable)
    {
        _correlate = enable;
    }

    void integrator::set_biases
    (
        const std::vector<double>& acc_bias,
        const std::vector<double>& gyr_bias
    )
    {
        if (acc_bias.size() == 3)
            _acc_bias = acc_bias;
        if (gyr_bias.size() == 3)
            _gyr_bias = gyr_bias;
    }

    auto integrator::integrate(const std::vector<frame::imu>& measurements, const double start, const double end) -> ugpm::PreintMeas
    {
        // first add all new measurements to _measurements
        for (const auto& m : measurements)
        {
            _measurements.emplace_back(m);
        }

        if (_measurements.empty() || start >= end)
        {
            SPDLOG_WARN("No IMU measurements to integrate or invalid time interval [{:.4f}, {:.4f}]", start, end);
            return ugpm::PreintMeas { };
        }

        constexpr int overlap_factor = 8;
        const auto    overlap_period = (end - start) * overlap_factor; // 8 times of period as advised

        std::vector<frame::imu> to_integrate = { };
        for (const auto& m : _measurements)
        {
            if (start - overlap_period <= m.timestamp && m.timestamp <= end)
            {
                to_integrate.push_back(m);
            }
        }

        erase_if(_measurements, [&](auto& m) { return m.timestamp < start - overlap_period; });

        ugpm::ImuData imu_data = { };
        imu_data.acc.reserve(to_integrate.size());
        imu_data.gyr.reserve(to_integrate.size());

        for (const auto& m : to_integrate)
        {
            imu_data.acc.emplace_back(ugpm::ImuSample { m.timestamp, m.acc[0], m.acc[1], m.acc[2] });
            imu_data.gyr.emplace_back(ugpm::ImuSample { m.timestamp, m.gyr[0], m.gyr[1], m.gyr[2] });
        }

        // Setup preintegration options
        ugpm::PreintOption options = { };
        options.type               = _method == method::ugpm ? ugpm::UGPM : ugpm::LPM;
        options.state_freq         = _state_freq;
        options.correlate          = _correlate;
        options.quantum            = -1; // Disable per-chunk mode

        ugpm::ImuPreintegration integration = { imu_data, start, end, options, _prior, false, overlap_factor };

        _prior = integration.getPrior();

        return integration.get();
    }

#else

    // Stub implementation when ugpm is not available
    preint::preint(const imu_calibration& imu_calib, method preint_method)
        : _dummy(0)
    {
        (void)imu_calib;
        (void)preint_method;
        SPDLOG_WARN("UGPM not available at compile time - IMU preintegration disabled");
    }

    void preint::set_overlap_factor(int factor) { (void)factor; }
    void preint::set_state_frequency(double freq) { (void)freq; }
    void preint::set_correlate(bool enable) { (void)enable; }

    void preint::set_biases
    (
        const std::vector<double>& acc_bias,
        const std::vector<double>& gyr_bias
    )
    {
        (void)acc_bias;
        (void)gyr_bias;
    }

#endif
} // namespace zenslam