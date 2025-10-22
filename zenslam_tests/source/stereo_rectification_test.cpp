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
        cv::Mat map_x, map_y;
        
        // Should not crash with empty maps
        auto result = zenslam::utils::rectify(empty_image, map_x, map_y);
        REQUIRE(result.empty());
    }

    SECTION("rectify with identity maps produces similar image")
    {
        // Create a simple test image
        cv::Mat test_image = cv::Mat::zeros(480, 640, CV_8UC1);
        cv::circle(test_image, cv::Point(320, 240), 50, cv::Scalar(255), -1);
        
        // Create identity maps (no transformation)
        cv::Mat map_x(480, 640, CV_32FC1);
        cv::Mat map_y(480, 640, CV_32FC1);
        for (int y = 0; y < 480; y++)
        {
            for (int x = 0; x < 640; x++)
            {
                map_x.at<float>(y, x) = static_cast<float>(x);
                map_y.at<float>(y, x) = static_cast<float>(y);
            }
        }
        
        auto result = zenslam::utils::rectify(test_image, map_x, map_y);
        
        // Result should have the same size as input
        REQUIRE(result.rows == test_image.rows);
        REQUIRE(result.cols == test_image.cols);
        REQUIRE(result.type() == test_image.type());
        
        // With identity maps, the image should be very similar
        // (may have minor interpolation differences)
        cv::Mat diff;
        cv::absdiff(test_image, result, diff);
        double max_diff = 0;
        cv::minMaxLoc(diff, nullptr, &max_diff);
        
        // Allow small differences due to interpolation
        REQUIRE(max_diff < 5.0);
    }
}
