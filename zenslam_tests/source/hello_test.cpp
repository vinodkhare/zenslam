#define CATCH_CONFIG_MAIN

#include <catch2/catch_all.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/core/types.hpp>

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