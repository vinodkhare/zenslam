/**
 * @file hough_example.cpp
 * @brief Example of using OpenCV's Hough Line Transform
 * 
 * Demonstrates both standard and probabilistic Hough transforms.
 * Part of core OpenCV, no contrib modules required.
 * 
 * Compilation:
 *   g++ -std=c++17 hough_example.cpp -o hough_example \
 *       `pkg-config --cflags --libs opencv4`
 * 
 * Or with vcpkg:
 *   g++ -std=c++17 hough_example.cpp -o hough_example \
 *       -I${VCPKG_ROOT}/installed/x64-linux/include \
 *       -L${VCPKG_ROOT}/installed/x64-linux/lib \
 *       -lopencv_core -lopencv_imgproc -lopencv_imgcodecs
 */

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>
#include <chrono>
#include <vector>

int main(int argc, char** argv)
{
    // Load image (grayscale)
    std::string image_path = (argc > 1) ? argv[1] : "test_image.png";
    cv::Mat image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
    
    if (image.empty()) {
        std::cerr << "Error: Could not load image from " << image_path << "\n";
        std::cerr << "Usage: " << argv[0] << " [image_path]\n";
        return 1;
    }
    
    std::cout << "Image size: " << image.cols << "x" << image.rows << "\n";
    
    // ========================================================================
    // Part 1: Edge Detection (Required for Hough)
    // ========================================================================
    
    std::cout << "\n=== Edge Detection ===\n";
    
    // Apply Gaussian blur to reduce noise
    cv::Mat blurred;
    cv::GaussianBlur(image, blurred, cv::Size(5, 5), 1.5);
    
    // Canny edge detection
    cv::Mat edges;
    double canny_low = 50.0;
    double canny_high = 150.0;
    
    auto t_start = std::chrono::high_resolution_clock::now();
    cv::Canny(blurred, edges, canny_low, canny_high);
    auto t_end = std::chrono::high_resolution_clock::now();
    
    auto duration_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    
    std::cout << "Edge detection time: " << duration_ms << " ms\n";
    
    // Count edge pixels
    int edge_pixels = cv::countNonZero(edges);
    std::cout << "Edge pixels: " << edge_pixels << " / " 
              << (image.rows * image.cols) << " (" 
              << (100.0 * edge_pixels / (image.rows * image.cols)) << "%)\n";
    
    // ========================================================================
    // Part 2: Standard Hough Transform (Detects Lines, Not Segments)
    // ========================================================================
    
    std::cout << "\n=== Standard Hough Transform ===\n";
    
    std::vector<cv::Vec2f> hough_lines;  // (rho, theta)
    double rho = 1.0;                    // Distance resolution (pixels)
    double theta = CV_PI / 180.0;        // Angle resolution (1 degree)
    int threshold = 100;                  // Minimum votes
    
    t_start = std::chrono::high_resolution_clock::now();
    cv::HoughLines(edges, hough_lines, rho, theta, threshold);
    t_end = std::chrono::high_resolution_clock::now();
    
    duration_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    
    std::cout << "Detected lines: " << hough_lines.size() << "\n";
    std::cout << "Detection time: " << duration_ms << " ms\n";
    
    // Note: Standard Hough returns infinite lines (rho, theta)
    // Need to compute endpoints for visualization
    std::cout << "\nFirst 5 lines (rho, theta):\n";
    for (size_t i = 0; i < std::min(size_t(5), hough_lines.size()); ++i) {
        float rho = hough_lines[i][0];
        float theta = hough_lines[i][1];
        std::cout << "  Line " << i << ": rho=" << rho 
                  << ", theta=" << (theta * 180.0 / CV_PI) << "°\n";
    }
    
    // ========================================================================
    // Part 3: Probabilistic Hough Transform (Detects Segments)
    // ========================================================================
    
    std::cout << "\n=== Probabilistic Hough Transform ===\n";
    
    std::vector<cv::Vec4i> segments;     // (x1, y1, x2, y2)
    double rho_p = 1.0;                  // Distance resolution
    double theta_p = CV_PI / 180.0;      // Angle resolution
    int threshold_p = 50;                // Minimum votes
    double minLineLength = 30.0;         // Minimum segment length
    double maxLineGap = 10.0;            // Maximum gap to merge segments
    
    t_start = std::chrono::high_resolution_clock::now();
    cv::HoughLinesP(edges, segments, rho_p, theta_p, threshold_p, 
                    minLineLength, maxLineGap);
    t_end = std::chrono::high_resolution_clock::now();
    
    duration_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    
    std::cout << "Detected segments: " << segments.size() << "\n";
    std::cout << "Detection time: " << duration_ms << " ms\n";
    std::cout << "Detection rate: " << (1000.0 / duration_ms) << " fps\n";
    
    std::cout << "\nFirst 5 segments:\n";
    for (size_t i = 0; i < std::min(size_t(5), segments.size()); ++i) {
        const auto& seg = segments[i];
        float length = std::sqrt(
            std::pow(seg[2] - seg[0], 2) + 
            std::pow(seg[3] - seg[1], 2)
        );
        float angle = std::atan2(seg[3] - seg[1], seg[2] - seg[0]) * 180.0f / CV_PI;
        
        std::cout << "  Segment " << i << ": "
                  << "(" << seg[0] << "," << seg[1] << ") -> "
                  << "(" << seg[2] << "," << seg[3] << "), "
                  << "length=" << length << ", angle=" << angle << "°\n";
    }
    
    // ========================================================================
    // Part 4: Parameter Sensitivity Test
    // ========================================================================
    
    std::cout << "\n=== Parameter Sensitivity ===\n";
    
    // Test different thresholds
    std::vector<int> thresholds = {30, 50, 70, 100};
    for (int thresh : thresholds) {
        std::vector<cv::Vec4i> test_segments;
        cv::HoughLinesP(edges, test_segments, 1.0, CV_PI/180.0, 
                       thresh, 30.0, 10.0);
        std::cout << "Threshold " << thresh << ": " 
                  << test_segments.size() << " segments\n";
    }
    
    // Test different minimum lengths
    std::cout << "\nMinimum length sensitivity:\n";
    std::vector<double> min_lengths = {10.0, 30.0, 50.0, 70.0};
    for (double min_len : min_lengths) {
        std::vector<cv::Vec4i> test_segments;
        cv::HoughLinesP(edges, test_segments, 1.0, CV_PI/180.0, 
                       50, min_len, 10.0);
        std::cout << "Min length " << min_len << ": " 
                  << test_segments.size() << " segments\n";
    }
    
    // ========================================================================
    // Part 5: Visualization
    // ========================================================================
    
    std::cout << "\n=== Visualization ===\n";
    
    // Visualize standard Hough lines
    cv::Mat image_hough;
    cv::cvtColor(image, image_hough, cv::COLOR_GRAY2BGR);
    
    for (const auto& line : hough_lines) {
        float rho = line[0];
        float theta = line[1];
        
        // Convert (rho, theta) to endpoints
        double a = std::cos(theta);
        double b = std::sin(theta);
        double x0 = a * rho;
        double y0 = b * rho;
        
        // Extend line across image
        cv::Point pt1(
            cvRound(x0 + 1000 * (-b)),
            cvRound(y0 + 1000 * (a))
        );
        cv::Point pt2(
            cvRound(x0 - 1000 * (-b)),
            cvRound(y0 - 1000 * (a))
        );
        
        cv::line(image_hough, pt1, pt2, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    }
    
    // Visualize probabilistic Hough segments
    cv::Mat image_segments;
    cv::cvtColor(image, image_segments, cv::COLOR_GRAY2BGR);
    
    for (const auto& seg : segments) {
        cv::Point p1(seg[0], seg[1]);
        cv::Point p2(seg[2], seg[3]);
        
        // Color by length
        float length = std::sqrt(
            std::pow(seg[2] - seg[0], 2) + 
            std::pow(seg[3] - seg[1], 2)
        );
        
        cv::Scalar color;
        if (length < 40) {
            color = cv::Scalar(0, 255, 0);  // Green (short)
        } else if (length < 80) {
            color = cv::Scalar(0, 255, 255);  // Yellow (medium)
        } else {
            color = cv::Scalar(0, 0, 255);  // Red (long)
        }
        
        cv::line(image_segments, p1, p2, color, 2, cv::LINE_AA);
    }
    
    std::cout << "Visualization created. Press any key to close windows.\n";
    
    // Display
    cv::imshow("Original Image", image);
    cv::imshow("Edges (Canny)", edges);
    cv::imshow("Standard Hough Lines", image_hough);
    cv::imshow("Probabilistic Hough Segments", image_segments);
    cv::waitKey(0);
    
    // Save outputs
    cv::imwrite("hough_edges.png", edges);
    cv::imwrite("hough_lines.png", image_hough);
    cv::imwrite("hough_segments.png", image_segments);
    std::cout << "Outputs saved.\n";
    
    return 0;
}

/*
 * Expected Output:
 * ================
 * Image size: 640x480
 * 
 * === Edge Detection ===
 * Edge detection time: 3-8 ms
 * Edge pixels: 20000 / 307200 (6.5%)
 * 
 * === Standard Hough Transform ===
 * Detected lines: 50-150
 * Detection time: 5-15 ms
 * 
 * First 5 lines (rho, theta):
 *   Line 0: rho=200, theta=0°
 *   Line 1: rho=150, theta=90°
 *   ...
 * 
 * === Probabilistic Hough Transform ===
 * Detected segments: 100-300
 * Detection time: 8-20 ms
 * Detection rate: 50-125 fps
 * 
 * First 5 segments:
 *   Segment 0: (100,50) -> (200,50), length=100, angle=0°
 *   ...
 * 
 * === Parameter Sensitivity ===
 * Threshold 30: 250 segments
 * Threshold 50: 180 segments
 * Threshold 70: 120 segments
 * Threshold 100: 80 segments
 * 
 * Minimum length sensitivity:
 * Min length 10.0: 300 segments
 * Min length 30.0: 180 segments
 * Min length 50.0: 100 segments
 * Min length 70.0: 60 segments
 * 
 * === Visualization ===
 * Visualization created. Press any key to close windows.
 * Outputs saved.
 * 
 * Notes:
 * ======
 * - Standard Hough vs Probabilistic Hough:
 *   * Standard: Returns infinite lines (rho, theta)
 *   * Probabilistic: Returns line segments (x1,y1,x2,y2)
 *   * Probabilistic is usually preferred for SLAM
 * 
 * - Key parameters:
 *   * threshold: Minimum votes in Hough space
 *   * minLineLength: Minimum segment length (pixels)
 *   * maxLineGap: Max gap to merge segments
 * 
 * - Performance:
 *   * Fast: 50-125 fps typical
 *   * Depends heavily on edge count
 *   * Edge detection is often the bottleneck
 * 
 * - Advantages:
 *   * Very fast
 *   * Part of core OpenCV (no contrib)
 *   * Well-established, robust
 *   * Good for structured scenes
 * 
 * - Disadvantages:
 *   * Less accurate endpoints than LSD/FLD
 *   * Sensitive to parameter tuning
 *   * Many false positives in textured regions
 *   * No built-in descriptors
 * 
 * - For ZenSLAM:
 *   * Not recommended as primary detector
 *   * Useful for validation/comparison
 *   * Good for simple structured environments
 *   * Consider for initial prototyping
 * 
 * - Parameter Tuning Guidelines:
 *   * threshold: Start with 50, increase to reduce false positives
 *   * minLineLength: Match your application (30-50 typical)
 *   * maxLineGap: 5-15 pixels typical
 *   * Canny thresholds: 50/150 is a good starting point
 */
