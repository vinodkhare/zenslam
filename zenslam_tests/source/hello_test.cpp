#define CATCH_CONFIG_MAIN

#include <catch2/catch_all.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/core/types.hpp>

#include "zenslam/detection/detector.h"
#include "zenslam/all_options.h"

TEST_CASE (
"Hello World Test"
)
{
    REQUIRE(1 + 1 == 2);
    REQUIRE(std::string("Hello") + " World" == "Hello World");
}

TEST_CASE("cv::Mat vs cv::UMat Keypoint Detection Benchmark", "[benchmark]")
{
    // Create a test image (1280x720, grayscale)
    const cv::Size image_size(1280, 720);
    cv::Mat test_image(image_size, CV_8UC1);
    cv::randu(test_image, 0, 255);
    
    // Add some features to detect
    for (int i = 0; i < 100; ++i)
    {
        cv::circle(test_image, 
                   cv::Point(rand() % image_size.width, rand() % image_size.height), 
                   5, cv::Scalar(255), -1);
    }
    
    // Create feature detector and descriptor
    auto detector = cv::ORB::create(500);
    
    SECTION("Benchmark cv::Mat (CPU)")
    {
        BENCHMARK("ORB detect + compute with cv::Mat")
        {
            const auto img = test_image.clone();
            std::vector<cv::KeyPoint> keypoints;
            cv::Mat descriptors;
            
            detector->detectAndCompute(img, cv::noArray(), keypoints, descriptors);
            
            return keypoints.size();
        };
    }

    // Unfortunately OpenCL is not available on MacOS!
    SECTION("Benchmark cv::UMat (OpenCL)")
    {
        // Check if OpenCL is available
        if (!cv::ocl::haveOpenCL())
        {
            WARN("OpenCL not available, skipping UMat benchmark");
            return;
        }
        
        cv::ocl::setUseOpenCL(true);
        REQUIRE(cv::ocl::useOpenCL());
        
        BENCHMARK("ORB detect + compute with cv::UMat")
        {
            cv::UMat uimg;
            test_image.copyTo(uimg);
            
            std::vector<cv::KeyPoint> keypoints;
            cv::UMat descriptors;
            
            detector->detectAndCompute(uimg, cv::noArray(), keypoints, descriptors);
            
            return keypoints.size();
        };
    }
}

TEST_CASE("cv::Mat vs cv::UMat Feature Matching Benchmark", "[benchmark]")
{
    // Create two test images
    const cv::Size image_size(1280, 720);
    cv::Mat img1(image_size, CV_8UC1);
    cv::Mat img2(image_size, CV_8UC1);
    cv::randu(img1, 0, 255);
    cv::randu(img2, 0, 255);
    
    // Add matching features
    for (int i = 0; i < 100; ++i)
    {
        cv::Point pt(rand() % image_size.width, rand() % image_size.height);
        cv::circle(img1, pt, 5, cv::Scalar(255), -1);
        cv::circle(img2, pt + cv::Point(rand() % 10 - 5, rand() % 10 - 5), 5, cv::Scalar(255), -1);
    }
    
    auto detector = cv::ORB::create(500);
    auto matcher = cv::BFMatcher::create(cv::NORM_HAMMING, true);
    
    SECTION("Benchmark cv::Mat (CPU) - Full Pipeline")
    {
        BENCHMARK("Detect + Compute + Match with cv::Mat")
        {
            std::vector<cv::KeyPoint> kp1, kp2;
            cv::Mat desc1, desc2;
            std::vector<cv::DMatch> matches;
            
            detector->detectAndCompute(img1, cv::noArray(), kp1, desc1);
            detector->detectAndCompute(img2, cv::noArray(), kp2, desc2);
            matcher->match(desc1, desc2, matches);
            
            return matches.size();
        };
    }
    
    SECTION("Benchmark cv::UMat (OpenCL) - Full Pipeline")
    {
        if (!cv::ocl::haveOpenCL())
        {
            WARN("OpenCL not available, skipping UMat benchmark");
            return;
        }
        
        cv::ocl::setUseOpenCL(true);
        
        BENCHMARK("Detect + Compute + Match with cv::UMat")
        {
            cv::UMat uimg1, uimg2;
            img1.copyTo(uimg1);
            img2.copyTo(uimg2);
            
            std::vector<cv::KeyPoint> kp1, kp2;
            cv::UMat desc1, desc2;
            std::vector<cv::DMatch> matches;
            
            detector->detectAndCompute(uimg1, cv::noArray(), kp1, desc1);
            detector->detectAndCompute(uimg2, cv::noArray(), kp2, desc2);
            matcher->match(desc1, desc2, matches);
            
            return matches.size();
        };
    }
}

TEST_CASE("grid_detector::detect_keypoints vs detect_keypoints_par Benchmark", "[benchmark]")
{
    // Create a realistic test image (1920x1080, grayscale)
    const cv::Size image_size(1920, 1080);
    cv::Mat test_image(image_size, CV_8UC1);
    cv::randu(test_image, 0, 255);
    
    // Add realistic scene features (corners, circles, lines)
    for (int i = 0; i < 200; ++i)
    {
        cv::Point center(rand() % image_size.width, rand() % image_size.height);
        int radius = 3 + rand() % 8;
        cv::circle(test_image, center, radius, cv::Scalar(255), -1);
    }
    
    // Add some line features
    for (int i = 0; i < 50; ++i)
    {
        cv::Point pt1(rand() % image_size.width, rand() % image_size.height);
        cv::Point pt2(rand() % image_size.width, rand() % image_size.height);
        cv::line(test_image, pt1, pt2, cv::Scalar(128), 2);
    }
    
    // Create grid_detector with typical SLAM settings
    zenslam::slam_options opts;
    opts.feature_detector = zenslam::feature_type::FAST;
    opts.descriptor       = zenslam::descriptor_type::ORB;
    opts.detection.fast_threshold = 10;
    opts.detection.cell_size      = cv::Size(64, 64);  // ~30x17 grid cells for 1920x1080
    
    auto detector = zenslam::detector::create(opts);
    
    // Empty existing keypoints map (detecting all new keypoints)
    zenslam::map<zenslam::keypoint> existing_keypoints;
    
    SECTION("Benchmark Sequential detect_keypoints")
    {
        BENCHMARK("grid_detector::detect_keypoints (sequential)")
        {
            auto keypoints = detector.detect_keypoints(test_image, existing_keypoints);
            return keypoints.size();
        };
    }
    
    SECTION("Benchmark Parallel detect_keypoints_par")
    {
        BENCHMARK("grid_detector::detect_keypoints_par (parallel)")
        {
            auto keypoints = detector.detect_keypoints_par(test_image, existing_keypoints);
            return keypoints.size();
        };
    }
    
    SECTION("Benchmark with Pre-existing Keypoints (Partial Occupancy)")
    {
        // Simulate scenario where 50% of grid cells already have keypoints
        zenslam::map<zenslam::keypoint> partial_keypoints;
        
        // Detect once to get real keypoints
        auto initial_keypoints = detector.detect_keypoints(test_image, existing_keypoints);
        
        // Add half of them to "existing" map using operator+=
        for (size_t i = 0; i < initial_keypoints.size() / 2; ++i)
        {
            partial_keypoints += initial_keypoints[i];
        }
        
        BENCHMARK("Sequential with 50% occupancy")
        {
            auto keypoints = detector.detect_keypoints(test_image, partial_keypoints);
            return keypoints.size();
        };
        
        BENCHMARK("Parallel with 50% occupancy")
        {
            auto keypoints = detector.detect_keypoints_par(test_image, partial_keypoints);
            return keypoints.size();
        };
    }
}