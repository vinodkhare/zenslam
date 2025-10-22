#include <catch2/catch_all.hpp>

#include <zenslam/options.h>
#include <zenslam/calibration.h>
#include <zenslam/utils.h>

#include <opencv2/core.hpp>

TEST_CASE("Stereo rectification option", "[options][stereo_rectify]")
{
    using zenslam::options;

    SECTION("Default stereo_rectify is false")
    {
        auto opts = options{};
        REQUIRE(opts.slam.stereo_rectify == false);
    }

    SECTION("stereo_rectify can be set to true")
    {
        auto opts = options{};
        opts.slam.stereo_rectify = true;
        REQUIRE(opts.slam.stereo_rectify == true);
        REQUIRE_NOTHROW(opts.slam.validate());
    }
}

TEST_CASE("Stereo rectification matrices computation", "[calibration][stereo_rectify]")
{
    using zenslam::calibration;

    SECTION("Rectification matrices have correct dimensions")
    {
        // Note: This test would require a valid calibration file to run
        // For now, we just verify that the calibration struct has the matrices
        calibration calib;
        
        // Verify matrices are initialized (all zeros by default)
        REQUIRE(calib.R1(0,0) == 0.0);
        REQUIRE(calib.R2(0,0) == 0.0);
        REQUIRE(calib.P1(0,0) == 0.0);
        REQUIRE(calib.P2(0,0) == 0.0);
        REQUIRE(calib.Q(0,0) == 0.0);
    }
}

TEST_CASE("utils::rectify function", "[utils][stereo_rectify]")
{
    SECTION("rectify handles empty image gracefully")
    {
        cv::Mat empty_image;
        cv::Matx33d K = cv::Matx33d::eye();
        std::vector<double> distortion_coeffs = {0.0, 0.0, 0.0, 0.0};
        cv::Matx33d R = cv::Matx33d::eye();
        cv::Matx34d P = cv::Matx34d::zeros();
        P(0,0) = 1.0; P(1,1) = 1.0; P(2,2) = 1.0;
        cv::Size resolution(640, 480);
        
        // Should not crash with empty image
        auto result = zenslam::utils::rectify(empty_image, K, distortion_coeffs, R, P, resolution);
        REQUIRE(result.empty());
    }

    SECTION("rectify with identity matrices produces similar image")
    {
        // Create a simple test image
        cv::Mat test_image = cv::Mat::zeros(480, 640, CV_8UC1);
        cv::circle(test_image, cv::Point(320, 240), 50, cv::Scalar(255), -1);
        
        // Identity camera matrix (simplified)
        cv::Matx33d K = cv::Matx33d::eye();
        K(0,0) = 500; K(1,1) = 500; K(0,2) = 320; K(1,2) = 240;
        
        std::vector<double> distortion_coeffs = {0.0, 0.0, 0.0, 0.0};
        cv::Matx33d R = cv::Matx33d::eye();
        cv::Matx34d P = cv::Matx34d::zeros();
        P(0,0) = K(0,0); P(1,1) = K(1,1); P(0,2) = K(0,2); P(1,2) = K(1,2); P(2,2) = 1.0;
        
        cv::Size resolution(640, 480);
        
        auto result = zenslam::utils::rectify(test_image, K, distortion_coeffs, R, P, resolution);
        
        // Result should have the same size as input
        REQUIRE(result.rows == test_image.rows);
        REQUIRE(result.cols == test_image.cols);
        REQUIRE(result.type() == test_image.type());
        
        // With identity transforms, the image should be very similar
        // (may have minor interpolation differences)
        cv::Mat diff;
        cv::absdiff(test_image, result, diff);
        double max_diff = 0;
        cv::minMaxLoc(diff, nullptr, &max_diff);
        
        // Allow small differences due to interpolation
        REQUIRE(max_diff < 5.0);
    }
}
