/**
 * @file lsd_example.cpp
 * @brief Example of using OpenCV's LSD (Line Segment Detector)
 * 
 * This is the current line detection method used in ZenSLAM.
 * 
 * Compilation (assuming OpenCV with contrib modules):
 *   g++ -std=c++17 lsd_example.cpp -o lsd_example \
 *       `pkg-config --cflags --libs opencv4`
 * 
 * Or with vcpkg:
 *   # Ensure opencv[contrib] is installed via vcpkg
 *   g++ -std=c++17 lsd_example.cpp -o lsd_example \
 *       -I${VCPKG_ROOT}/installed/x64-linux/include \
 *       -L${VCPKG_ROOT}/installed/x64-linux/lib \
 *       -lopencv_core -lopencv_imgproc -lopencv_imgcodecs \
 *       -lopencv_line_descriptor
 */

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/line_descriptor.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>
#include <chrono>
#include <vector>

namespace lcd = cv::line_descriptor;

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
    // Part 1: Line Segment Detection with LSD
    // ========================================================================
    
    // Create LSD detector
    cv::Ptr<lcd::LSDDetector> lsd = lcd::LSDDetector::createLSDDetector();
    
    std::cout << "\n=== Line Segment Detection ===\n";
    
    // Detect keylines
    std::vector<lcd::KeyLine> keylines;
    
    auto t_start = std::chrono::high_resolution_clock::now();
    lsd->detect(image, keylines, 
                2.0f,  // scale (2.0 = half resolution)
                1,     // numOctaves (1 = single scale)
                cv::noArray());  // mask (empty = detect everywhere)
    auto t_end = std::chrono::high_resolution_clock::now();
    
    auto duration_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    
    std::cout << "Detected lines: " << keylines.size() << "\n";
    std::cout << "Detection time: " << duration_ms << " ms\n";
    std::cout << "Detection rate: " << (1000.0 / duration_ms) << " fps\n";
    
    // Print information about first few lines
    std::cout << "\nFirst 5 detected lines:\n";
    for (size_t i = 0; i < std::min(size_t(5), keylines.size()); ++i) {
        const auto& kl = keylines[i];
        std::cout << "  Line " << i << ": "
                  << "(" << kl.startPointX << "," << kl.startPointY << ") -> "
                  << "(" << kl.endPointX << "," << kl.endPointY << "), "
                  << "length=" << kl.lineLength << ", "
                  << "angle=" << kl.angle << "°\n";
    }
    
    // ========================================================================
    // Part 2: Binary Descriptor Computation
    // ========================================================================
    
    std::cout << "\n=== Binary Descriptor Computation ===\n";
    
    // Create binary descriptor extractor
    cv::Ptr<lcd::BinaryDescriptor> bd = lcd::BinaryDescriptor::createBinaryDescriptor();
    
    // Compute descriptors
    cv::Mat descriptors;
    
    t_start = std::chrono::high_resolution_clock::now();
    bd->compute(image, keylines, descriptors);
    t_end = std::chrono::high_resolution_clock::now();
    
    duration_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    
    std::cout << "Computed descriptors: " << descriptors.rows << "\n";
    std::cout << "Descriptor size: " << descriptors.cols << " bytes\n";
    std::cout << "Computation time: " << duration_ms << " ms\n";
    
    // ========================================================================
    // Part 3: Detection with Mask (Selective Detection)
    // ========================================================================
    
    std::cout << "\n=== Detection with Mask ===\n";
    
    // Create a mask that blocks the center region
    cv::Mat mask = cv::Mat::ones(image.size(), CV_8U) * 255;
    int center_x = image.cols / 2;
    int center_y = image.rows / 2;
    int block_width = image.cols / 4;
    int block_height = image.rows / 4;
    
    cv::Rect roi(center_x - block_width/2, center_y - block_height/2, 
                 block_width, block_height);
    mask(roi) = 0;  // Block center region
    
    // Detect with mask
    std::vector<lcd::KeyLine> keylines_masked;
    lsd->detect(image, keylines_masked, 2.0f, 1, mask);
    
    std::cout << "Lines detected with mask: " << keylines_masked.size() << "\n";
    std::cout << "Lines excluded by mask: " << (keylines.size() - keylines_masked.size()) << "\n";
    
    // ========================================================================
    // Part 4: Visualization
    // ========================================================================
    
    std::cout << "\n=== Visualization ===\n";
    
    // Convert grayscale to BGR for colored visualization
    cv::Mat image_color;
    cv::cvtColor(image, image_color, cv::COLOR_GRAY2BGR);
    
    // Draw detected lines
    for (const auto& kl : keylines) {
        cv::Point start(static_cast<int>(kl.startPointX),
                       static_cast<int>(kl.startPointY));
        cv::Point end(static_cast<int>(kl.endPointX),
                     static_cast<int>(kl.endPointY));
        
        // Color by length: short=green, medium=yellow, long=red
        cv::Scalar color;
        if (kl.lineLength < 30) {
            color = cv::Scalar(0, 255, 0);  // Green
        } else if (kl.lineLength < 60) {
            color = cv::Scalar(0, 255, 255);  // Yellow
        } else {
            color = cv::Scalar(0, 0, 255);  // Red
        }
        
        cv::line(image_color, start, end, color, 2, cv::LINE_AA);
    }
    
    std::cout << "Visualization created. Press any key to close windows.\n";
    
    // Display
    cv::imshow("Original Image", image);
    cv::imshow("Detected Lines (colored by length)", image_color);
    cv::imshow("Detection Mask", mask);
    cv::waitKey(0);
    
    // Save output
    std::string output_path = "lsd_output.png";
    cv::imwrite(output_path, image_color);
    std::cout << "Output saved to: " << output_path << "\n";
    
    return 0;
}

/*
 * Expected Output:
 * ================
 * Image size: 640x480
 * 
 * === Line Segment Detection ===
 * Detected lines: 150-300 (depends on image)
 * Detection time: 20-40 ms (depends on hardware)
 * Detection rate: 25-50 fps
 * 
 * First 5 detected lines:
 *   Line 0: (100,50) -> (200,50), length=100, angle=0°
 *   Line 1: (150,80) -> (150,180), length=100, angle=90°
 *   ...
 * 
 * === Binary Descriptor Computation ===
 * Computed descriptors: 150-300
 * Descriptor size: 32 bytes
 * Computation time: 10-20 ms
 * 
 * === Detection with Mask ===
 * Lines detected with mask: 100-200
 * Lines excluded by mask: 50-100
 * 
 * === Visualization ===
 * Visualization created. Press any key to close windows.
 * Output saved to: lsd_output.png
 * 
 * Notes:
 * ======
 * - LSD parameters:
 *   * scale=2.0: Process image at half resolution for speed
 *   * numOctaves=1: Single-scale detection
 *   * Use scale=1.0 for full resolution detection
 * 
 * - Binary descriptors are 32 bytes (256 bits)
 * - Fast matching possible with Hamming distance
 * 
 * - Performance characteristics:
 *   * Detection: O(N) where N = number of pixels
 *   * Descriptor computation: O(M*K) where M = lines, K = descriptor size
 *   * Typical bottleneck: Detection phase
 * 
 * - For ZenSLAM integration:
 *   * Use in grid_detector class (already implemented)
 *   * Combine with masking for tracked lines
 *   * Use binary descriptors for stereo matching
 */
