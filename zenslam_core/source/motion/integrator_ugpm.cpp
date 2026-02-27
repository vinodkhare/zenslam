// Separate compilation unit for UGPM preintegration to isolate expensive template instantiations
// This reduces compile-time by preventing excessive inlining across translation units

#include "zenslam/motion/integrator_types.h"
#include "zenslam/frame/imu.h"
#include "zenslam/calibration/imu_calibration.h"

#include <preint/preint.h>
#include <vector>

namespace zenslam::detail
{
    // Mark as noinline to prevent excessive code bloat from template expansion
    [[msvc::noinline, gnu::noinline]]
    integral integrate_ugpm_impl(
        const std::vector<frame::imu>& measurements,
        const double start,
        const double end,
        const int overlap_factor,
        const ugpm::PreintType preint_type,
        const ugpm::PreintPrior& prior_in,
        ugpm::PreintPrior& prior_out)
    {
        ugpm::ImuData imu_data = {};
        imu_data.acc.reserve(measurements.size());
        imu_data.gyr.reserve(measurements.size());

        for (const auto& [timestamp, acc, gyr] : measurements)
        {
            imu_data.acc.emplace_back(ugpm::ImuSample{ timestamp, acc[0], acc[1], acc[2] });
            imu_data.gyr.emplace_back(ugpm::ImuSample{ timestamp, gyr[0], gyr[1], gyr[2] });
        }

        // Setup preintegration options
        ugpm::PreintOption options = {};
        options.type = preint_type;
        options.state_freq = 200;
        options.correlate = true;
        options.quantum = -1; // Disable per-chunk mode

        ugpm::ImuPreintegration integration = { imu_data, start, end, options, prior_in, false, overlap_factor };

        prior_out = integration.getPrior();

        const auto m = integration.get();

        // Convert to zenslam format
        integral z{};
        
        // Helper lambdas for conversion
        auto eigenToMatx33d = [](const Eigen::Matrix3d& M) {
            return cv::Matx33d{
                M(0, 0), M(0, 1), M(0, 2),
                M(1, 0), M(1, 1), M(1, 2),
                M(2, 0), M(2, 1), M(2, 2)
            };
        };

        auto eigenToVec3d = [](const Eigen::Vector3d& v) {
            return cv::Vec3d{ v(0), v(1), v(2) };
        };

        auto eigenToMatx9x9 = [](const Eigen::Matrix<double, 9, 9>& M) {
            cv::Matx<double, 9, 9> out;
            for (auto r = 0; r < 9; ++r)
                for (auto c = 0; c < 9; ++c)
                    out(r, c) = M(r, c);
            return out;
        };

        z.delta_R = eigenToMatx33d(m.delta_R);
        z.delta_v = eigenToVec3d(m.delta_v);
        z.delta_p = eigenToVec3d(m.delta_p);
        z.dt = m.dt;
        z.dt_sq_half = m.dt_sq_half;
        z.cov = eigenToMatx9x9(m.cov);

        return z;
    }
} // namespace zenslam::detail
