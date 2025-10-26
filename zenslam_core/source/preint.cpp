#include "zenslam/preint.h"

#include <opencv2/core/affine.hpp>
#include <spdlog/spdlog.h>

namespace zenslam
{

#if ZENSLAM_HAS_UGPM

preint::preint(const imu_calibration& imu_calib, method preint_method)
    : _imu_calib(imu_calib)
    , _method(preint_method)
    , _overlap_factor(8)  // Default: 8x state period for optimal UGPM
    , _state_freq(50.0)   // Default: 50 Hz state frequency
    , _correlate(true)    // Default: enable correlation
    , _acc_bias({0.0, 0.0, 0.0})
    , _gyr_bias({0.0, 0.0, 0.0})
    , _prev_end_time(0.0)
{
}

void preint::set_overlap_factor(int factor)
{
    _overlap_factor = factor;
}

void preint::set_state_frequency(double freq)
{
    _state_freq = freq;
}

void preint::set_correlate(bool enable)
{
    _correlate = enable;
}

void preint::set_biases(
    const std::vector<double>& acc_bias,
    const std::vector<double>& gyr_bias
)
{
    if (acc_bias.size() == 3)
        _acc_bias = acc_bias;
    if (gyr_bias.size() == 3)
        _gyr_bias = gyr_bias;
}

auto preint::to_ugpm_data(
    const std::vector<frame::imu_measurement>& measurements
) -> ugpm::ImuData
{
    ugpm::ImuData imu_data;
    
    // Set noise variances from calibration (variance = density^2)
    imu_data.acc_var = _imu_calib.accelerometer_noise_density * 
                       _imu_calib.accelerometer_noise_density;
    imu_data.gyr_var = _imu_calib.gyroscope_noise_density * 
                       _imu_calib.gyroscope_noise_density;

    imu_data.acc.reserve(measurements.size());
    imu_data.gyr.reserve(measurements.size());

    for (const auto& m : measurements)
    {
        ugpm::ImuSample acc_s{};
        acc_s.t       = m.timestamp;
        acc_s.data[0] = m.alpha_x;
        acc_s.data[1] = m.alpha_y;
        acc_s.data[2] = m.alpha_z;
        imu_data.acc.push_back(acc_s);

        ugpm::ImuSample gyr_s{};
        gyr_s.t       = m.timestamp;
        gyr_s.data[0] = m.omega_x;
        gyr_s.data[1] = m.omega_y;
        gyr_s.data[2] = m.omega_z;
        imu_data.gyr.push_back(gyr_s);
    }

    return imu_data;
}

auto preint::integrate(
    const std::vector<frame::imu_measurement>& measurements,
    double start_time,
    double end_time
) -> std::optional<ugpm::PreintMeas>
{
    if (measurements.empty())
    {
        SPDLOG_DEBUG("No IMU measurements to integrate");
        return identity();
    }

    if (end_time <= start_time)
    {
        SPDLOG_WARN("Invalid integration interval: end_time ({}) <= start_time ({})", 
                    end_time, start_time);
        return identity();
    }

    try
    {
        // Convert measurements to ugpm format
        auto imu_data = to_ugpm_data(measurements);

        // Setup preintegration options
        ugpm::PreintOption opt;
        opt.type      = (_method == method::ugpm) ? ugpm::UGPM : ugpm::LPM;
        opt.state_freq = _state_freq;
        opt.correlate = _correlate;
        opt.quantum   = -1;  // Disable per-chunk mode

        // Setup prior biases
        ugpm::PreintPrior prior;
        prior.acc_bias = _acc_bias;
        prior.gyr_bias = _gyr_bias;

        // Compute overlap value for UGPM
        const int overlap = _overlap_factor;  // ugpm uses this internally

        // Perform preintegration
        ugpm::ImuPreintegration preintegrator(
            imu_data, 
            start_time, 
            end_time, 
            opt, 
            prior,
            false,    // rot_only = false (compute full preintegration)
            overlap   // overlap factor for optimal UGPM performance
        );

        auto result = preintegrator.get();
        
        SPDLOG_DEBUG("IMU preintegration completed: dt={:.4f}s, |delta_v|={:.4f}m/s, |delta_p|={:.4f}m",
                    result.dt,
                    result.delta_v.norm(),
                    result.delta_p.norm());

        return result;
    }
    catch (const std::exception& e)
    {
        SPDLOG_ERROR("IMU preintegration failed: {}", e.what());
        return std::nullopt;
    }
}

auto preint::integrate_with_overlap(
    const std::vector<frame::imu_measurement>& measurements,
    double start_time,
    double end_time
) -> std::optional<ugpm::PreintMeas>
{
    if (measurements.empty())
    {
        return identity();
    }

    // Compute overlap duration
    const double overlap_duration = _overlap_factor / _state_freq;
    const double overlap_start = std::max(0.0, start_time - overlap_duration);

    // Combine previous overlap data with current measurements
    std::vector<frame::imu_measurement> combined_measurements;

    // Add overlap data from previous window
    for (const auto& prev_m : _prev_measurements)
    {
        if (prev_m.timestamp >= overlap_start && prev_m.timestamp < start_time)
        {
            combined_measurements.push_back(prev_m);
        }
    }

    // Add current measurements
    combined_measurements.insert(
        combined_measurements.end(),
        measurements.begin(),
        measurements.end()
    );

    SPDLOG_DEBUG("Integrating with overlap: {} overlap + {} current = {} total measurements",
                combined_measurements.size() - measurements.size(),
                measurements.size(),
                combined_measurements.size());

    // Integrate with combined data
    auto result = integrate(combined_measurements, start_time, end_time);

    // Store current measurements for next overlap
    _prev_measurements = measurements;
    _prev_end_time = end_time;

    return result;
}

auto preint::identity() -> ugpm::PreintMeas
{
    ugpm::PreintMeas result;
    result.delta_R    = ugpm::Mat3::Identity();
    result.delta_v    = ugpm::Vec3::Zero();
    result.delta_p    = ugpm::Vec3::Zero();
    result.dt         = 0.0;
    result.dt_sq_half = 0.0;
    result.cov.setZero();
    
    // Initialize Jacobians to zero/identity
    result.d_delta_R_d_bw = ugpm::Mat3::Zero();
    result.d_delta_R_d_t  = ugpm::Vec3::Zero();
    result.d_delta_v_d_bw = ugpm::Mat3::Zero();
    result.d_delta_v_d_bf = ugpm::Mat3::Zero();
    result.d_delta_v_d_t  = ugpm::Vec3::Zero();
    result.d_delta_p_d_bw = ugpm::Mat3::Zero();
    result.d_delta_p_d_bf = ugpm::Mat3::Zero();
    result.d_delta_p_d_t  = ugpm::Vec3::Zero();

    return result;
}

auto preint::predict_pose(
    const cv::Affine3d& pose_0,
    const cv::Vec3d& velocity_0,
    const ugpm::PreintMeas& preint_meas,
    const cv::Vec3d& gravity
) -> cv::Affine3d
{
    // Extract rotation and translation from initial pose
    const cv::Matx33d R_0_cv = pose_0.rotation();
    const cv::Vec3d p_0 = pose_0.translation();
    const double dt = preint_meas.dt;

    // Convert ugpm Eigen matrices to OpenCV
    cv::Matx33d delta_R_cv;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            delta_R_cv(i, j) = preint_meas.delta_R(i, j);

    cv::Vec3d delta_v_cv(
        preint_meas.delta_v(0),
        preint_meas.delta_v(1),
        preint_meas.delta_v(2)
    );

    cv::Vec3d delta_p_cv(
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

    SPDLOG_DEBUG("IMU pose prediction: dt={:.4f}s, |delta_p|={:.4f}m, predicted_translation=[{:.4f}, {:.4f}, {:.4f}]m",
                dt,
                cv::norm(delta_p_cv),
                p_1[0], p_1[1], p_1[2]);

    return cv::Affine3d(R_1, p_1);
}

auto preint::predict_velocity(
    const cv::Affine3d& pose_0,
    const cv::Vec3d& velocity_0,
    const ugpm::PreintMeas& preint_meas,
    const cv::Vec3d& gravity
) -> cv::Vec3d
{
    // Extract rotation from initial pose
    const cv::Matx33d R_0_cv = pose_0.rotation();
    const double dt = preint_meas.dt;

    // Convert ugpm velocity delta to OpenCV
    cv::Vec3d delta_v_cv(
        preint_meas.delta_v(0),
        preint_meas.delta_v(1),
        preint_meas.delta_v(2)
    );

    // Predict velocity: v_1 = v_0 + R_0 * delta_v + g * dt
    cv::Vec3d v_1 = velocity_0 + R_0_cv * delta_v_cv + gravity * dt;

    SPDLOG_DEBUG("IMU velocity prediction: dt={:.4f}s, |delta_v|={:.4f}m/s, predicted_velocity=[{:.4f}, {:.4f}, {:.4f}]m/s",
                dt,
                cv::norm(delta_v_cv),
                v_1[0], v_1[1], v_1[2]);

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
void preint::set_biases(
    const std::vector<double>& acc_bias,
    const std::vector<double>& gyr_bias
)
{
    (void)acc_bias;
    (void)gyr_bias;
}

#endif

} // namespace zenslam
