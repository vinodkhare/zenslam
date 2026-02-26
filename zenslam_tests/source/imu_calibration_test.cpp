#include <filesystem>
#include <fstream>

#include <catch2/catch_all.hpp>

#include <zenslam/calibration/imu_calibration.h>
#include <zenslam/frame/imu.h>
#include <zenslam/motion/integrator.h>

TEST_CASE("IMU Calibration Parsing", "[imu_calibration]")
{
    // Create a temporary IMU config file for testing
    const std::string temp_file = "/tmp/test_imu_config.yaml";

    SECTION("Parse valid IMU config file")
    {
        // Create test file
        std::ofstream file(temp_file);
        file << "rostopic: /imu0\n";
        file << "update_rate: 200.0\n";
        file << "accelerometer_noise_density: 0.0028\n";
        file << "accelerometer_random_walk: 0.00086\n";
        file << "gyroscope_noise_density: 0.00016\n";
        file << "gyroscope_random_walk: 0.000022\n";
        file.close();

        // Parse the file
        auto [
                rostopic,
                update_rate,
                accelerometer_noise_density,
                accelerometer_random_walk,
                gyroscope_noise_density,
                gyroscope_random_walk
            ] =
            zenslam::imu_calibration::parse(temp_file);

        // Verify values
        REQUIRE(rostopic == "/imu0");
        REQUIRE(update_rate == Catch::Approx(200.0));
        REQUIRE(accelerometer_noise_density == Catch::Approx(0.0028));
        REQUIRE(accelerometer_random_walk == Catch::Approx(0.00086));
        REQUIRE(gyroscope_noise_density == Catch::Approx(0.00016));
        REQUIRE(gyroscope_random_walk == Catch::Approx(0.000022));

        // Clean up
        std::filesystem::remove(temp_file);
    }

    SECTION("Parse IMU config file with optional fields missing")
    {
        // Create test file without rostopic
        std::ofstream file(temp_file);
        file << "update_rate: 100.0\n";
        file << "accelerometer_noise_density: 0.001\n";
        file << "accelerometer_random_walk: 0.0001\n";
        file << "gyroscope_noise_density: 0.0001\n";
        file << "gyroscope_random_walk: 0.00001\n";
        file.close();

        // Parse the file
        auto imu_calib = zenslam::imu_calibration::parse(temp_file);

        // Verify values
        REQUIRE(imu_calib.rostopic.empty());
        REQUIRE(imu_calib.update_rate == Catch::Approx(100.0));
        REQUIRE(imu_calib.accelerometer_noise_density == Catch::Approx(0.001));

        // Clean up
        std::filesystem::remove(temp_file);
    }

    SECTION("Throw exception for non-existent file")
    {
        REQUIRE_THROWS_AS
        (
            zenslam::imu_calibration::parse("/non/existent/file.yaml"),
            std::runtime_error
        );
    }
}

TEST_CASE("IMU Preintegration Performance", "[imu_calibration][benchmark]")
{
    // Setup: Create IMU calibration (typical TUM-VI dataset values)
    zenslam::imu_calibration imu_calib;
    imu_calib.update_rate                 = 200.0; // 200 Hz IMU
    imu_calib.accelerometer_noise_density = 0.0028;
    imu_calib.accelerometer_random_walk   = 0.00086;
    imu_calib.gyroscope_noise_density     = 0.00016;
    imu_calib.gyroscope_random_walk       = 0.000022;

    // Generate synthetic IMU data: 200 Hz for 0.05 seconds = 10 samples
    // This represents one inter-frame interval at 20 Hz camera rate
    constexpr auto start_time = 0.0;
    constexpr auto end_time   = 0.05;        // 50ms integration window
    constexpr auto dt         = 1.0 / 200.0; // 5ms between samples

    std::vector<zenslam::frame::imu> imu_measurements;
    for (auto t = start_time; t <= end_time; t += dt)
    {
        zenslam::frame::imu measurement;
        measurement.timestamp = t;
        // Simulate constant angular velocity (rotating around z-axis)
        measurement.gyr = cv::Vec3d(0.0, 0.0, 0.1); // 0.1 rad/s
        // Simulate gravity + small perturbation
        measurement.acc = cv::Vec3d(0.1, 0.1, -9.81); // m/s²
        imu_measurements.push_back(measurement);
    }

    SECTION("Benchmark LPM vs Basalt")
    {
        BENCHMARK("LPM preintegration")
        {
            zenslam::integrator lpm_integrator(imu_calib, zenslam::integrator::method::lpm);
            return lpm_integrator.integrate(imu_measurements, start_time, end_time);
        };

        BENCHMARK("Basalt preintegration")
        {
            zenslam::integrator basalt_integrator(imu_calib, zenslam::integrator::method::basalt);
            return basalt_integrator.integrate(imu_measurements, start_time, end_time);
        };
    }

    SECTION("Extended benchmark: 100 samples (0.5s window)")
    {
        // Generate more samples for longer integration window
        std::vector<zenslam::frame::imu> extended_measurements;
        constexpr auto                   extended_end = 0.5; // 500ms window
        for (auto t = start_time; t <= extended_end; t += dt)
        {
            zenslam::frame::imu measurement;
            measurement.timestamp = t;
            measurement.gyr       = cv::Vec3d(0.05 * std::sin(t * 2.0), 0.0, 0.1);
            measurement.acc       = cv::Vec3d(0.1 * std::cos(t * 2.0), 0.1, -9.81);
            extended_measurements.push_back(measurement);
        }

        BENCHMARK("LPM preintegration (100 samples)")
        {
            zenslam::integrator lpm_integrator(imu_calib, zenslam::integrator::method::lpm);
            return lpm_integrator.integrate(extended_measurements, start_time, extended_end);
        };

        BENCHMARK("Basalt preintegration (100 samples)")
        {
            zenslam::integrator basalt_integrator(imu_calib, zenslam::integrator::method::basalt);
            return basalt_integrator.integrate(extended_measurements, start_time, extended_end);
        };
    }
}
