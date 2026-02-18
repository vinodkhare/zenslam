#include <filesystem>
#include <fstream>
#include <catch2/catch_all.hpp>
#include <zenslam/calibration/imu_calibration.h>

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
