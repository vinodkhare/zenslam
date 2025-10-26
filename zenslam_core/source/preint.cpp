#include "zenslam/preint.h"

#include <opencv2/core/affine.hpp>
#include <utility>
#include <spdlog/spdlog.h>

namespace zenslam
{
#if ZENSLAM_HAS_UGPM

    preint::preint(imu_calibration imu_calib, method preint_method)
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

    void preint::set_overlap_factor(const int factor)
    {
        _overlap_factor = factor;
    }

    void preint::set_state_frequency(const double freq)
    {
        _state_freq = freq;
    }

    void preint::set_correlate(const bool enable)
    {
        _correlate = enable;
    }

    void preint::set_biases
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

    auto preint::integrate(const std::vector<frame::imu>& measurements, const double start, const double end) -> ugpm::PreintMeas
    {
        // first add all new measurements to _measurements
        for (const auto& m : measurements)
        {
            _measurements.emplace_back(m);
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

        erase_if(_measurements, [&](auto& m){ return m.timestamp < start - overlap_period; });

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

    auto preint::predict_pose
    (
        const cv::Affine3d&     pose_0,
        const cv::Vec3d&        velocity_0,
        const ugpm::PreintMeas& preint_meas,
        const cv::Vec3d&        gravity
    ) -> cv::Affine3d
    {
        // Extract rotation and translation from initial pose
        const cv::Matx33d R_0_cv = pose_0.rotation();
        const cv::Vec3d   p_0    = pose_0.translation();
        const double      dt     = preint_meas.dt;

        // Convert ugpm Eigen matrices to OpenCV
        cv::Matx33d delta_R_cv;
        for (int i = 0; i < 3; ++i)
            for (int j           = 0; j < 3; ++j)
                delta_R_cv(i, j) = preint_meas.delta_R(i, j);

        cv::Vec3d delta_v_cv
        (
            preint_meas.delta_v(0),
            preint_meas.delta_v(1),
            preint_meas.delta_v(2)
        );

        cv::Vec3d delta_p_cv
        (
            preint_meas.delta_p(0),
            preint_meas.delta_p(1),
            preint_meas.delta_p(2)
        );

        // Predict rotation: R_1 = R_0 * delta_R
        cv::Matx33d R_1 = R_0_cv * delta_R_cv;

        // Predict position: p_1 = p_0 + v_0 * dt + R_0 * delta_p + 0.5 * g * dt^2
        cv::Vec3d p_1 = p_0 +
            velocity_0 * dt +
            R_0_cv * delta_p_cv +
            0.5 * gravity * dt * dt;

        SPDLOG_DEBUG
        (
            "IMU pose prediction: dt={:.4f}s, |delta_p|={:.4f}m, predicted_translation=[{:.4f}, {:.4f}, {:.4f}]m",
            dt,
            cv::norm(delta_p_cv),
            p_1[0],
            p_1[1],
            p_1[2]
        );

        return cv::Affine3d(R_1, p_1);
    }

    auto preint::predict_velocity
    (
        const cv::Affine3d&     pose_0,
        const cv::Vec3d&        velocity_0,
        const ugpm::PreintMeas& preint_meas,
        const cv::Vec3d&        gravity
    ) -> cv::Vec3d
    {
        // Extract rotation from initial pose
        const cv::Matx33d R_0_cv = pose_0.rotation();
        const double      dt     = preint_meas.dt;

        // Convert ugpm velocity delta to OpenCV
        cv::Vec3d delta_v_cv
        (
            preint_meas.delta_v(0),
            preint_meas.delta_v(1),
            preint_meas.delta_v(2)
        );

        // Predict velocity: v_1 = v_0 + R_0 * delta_v + g * dt
        cv::Vec3d v_1 = velocity_0 + R_0_cv * delta_v_cv + gravity * dt;

        SPDLOG_DEBUG
        (
            "IMU velocity prediction: dt={:.4f}s, |delta_v|={:.4f}m/s, predicted_velocity=[{:.4f}, {:.4f}, {:.4f}]m/s",
            dt,
            cv::norm(delta_v_cv),
            v_1[0],
            v_1[1],
            v_1[2]
        );

        return v_1;
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
