/**
 * @file fld_example.cpp
 * @brief Example of using OpenCV's FLD (Fast Line Detector)
 * 
 * FLD is faster than LSD and available in opencv_contrib ximgproc module.
 * This is a good vcpkg-friendly alternative to LSD.
 * 
 * Compilation (assuming OpenCV with contrib modules):
 *   g++ -std=c++17 fld_example.cpp -o fld_example \
 *       `pkg-config --cflags --libs opencv4`
 * 
 * Or with vcpkg:
 *   # Ensure opencv[contrib] is installed via vcpkg
 *   g++ -std=c++17 fld_example.cpp -o fld_example \
 *       -I${VCPKG_ROOT}/installed/x64-linux/include \
 *       -L${VCPKG_ROOT}/installed/x64-linux/lib \
 *       -lopencv_core -lopencv_imgproc -lopencv_imgcodecs \
 *       -lopencv_ximgproc
 */

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc.hpp>
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
    // Part 1: Basic FLD Detection with Default Parameters
    // ========================================================================
    
    std::cout << "\n=== FLD Detection (Default Parameters) ===\n";
    
    // Create FLD detector with default parameters
    cv::Ptr<cv::ximgproc::FastLineDetector> fld = 
        cv::ximgproc::createFastLineDetector();
    
    // Detect lines
    std::vector<cv::Vec4f> lines;
    
    auto t_start = std::chrono::high_resolution_clock::now();
    fld->detect(image, lines);
    auto t_end = std::chrono::high_resolution_clock::now();
    
    auto duration_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    
    std::cout << "Detected lines: " << lines.size() << "\n";
    std::cout << "Detection time: " << duration_ms << " ms\n";
    std::cout << "Detection rate: " << (1000.0 / duration_ms) << " fps\n";
    
    // Print information about first few lines
    std::cout << "\nFirst 5 detected lines:\n";
    for (size_t i = 0; i < std::min(size_t(5), lines.size()); ++i) {
        const auto& line = lines[i];
        float length = std::sqrt(
            std::pow(line[2] - line[0], 2) + 
            std::pow(line[3] - line[1], 2)
        );
        float angle = std::atan2(line[3] - line[1], line[2] - line[0]) * 180.0f / CV_PI;
        
        std::cout << "  Line " << i << ": "
                  << "(" << line[0] << "," << line[1] << ") -> "
                  << "(" << line[2] << "," << line[3] << "), "
                  << "length=" << length << ", "
                  << "angle=" << angle << "°\n";
    }
    
    // ========================================================================
    // Part 2: FLD Detection with Custom Parameters
    // ========================================================================
    
    std::cout << "\n=== FLD Detection (Custom Parameters) ===\n";
    
    // Create FLD detector with custom parameters
    cv::Ptr<cv::ximgproc::FastLineDetector> fld_custom = 
        cv::ximgproc::createFastLineDetector(
            10,     // length_threshold: minimum line length (pixels)
            1.41f,  // distance_threshold: max distance for merging segments
            50.0,   // canny_th1: first threshold for Canny
            50.0,   // canny_th2: second threshold for Canny
            3,      // canny_aperture_size: aperture size for Sobel
            false   // do_merge: merge collinear segments
        );
    
    std::vector<cv::Vec4f> lines_custom;
    
    t_start = std::chrono::high_resolution_clock::now();
    fld_custom->detect(image, lines_custom);
    t_end = std::chrono::high_resolution_clock::now();
    
    duration_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    
    std::cout << "Detected lines (custom): " << lines_custom.size() << "\n";
    std::cout << "Detection time: " << duration_ms << " ms\n";
    
    // ========================================================================
    // Part 3: FLD with Merging Enabled
    // ========================================================================
    
    std::cout << "\n=== FLD Detection (With Merging) ===\n";
    
    cv::Ptr<cv::ximgproc::FastLineDetector> fld_merge = 
        cv::ximgproc::createFastLineDetector(
            20,     // length_threshold: longer minimum length
            1.41f,  // distance_threshold
            50.0,   // canny_th1
            50.0,   // canny_th2
            3,      // canny_aperture_size
            true    // do_merge: merge collinear segments (reduces line count)
        );
    
    std::vector<cv::Vec4f> lines_merged;
    fld_merge->detect(image, lines_merged);
    
    std::cout << "Detected lines (merged): " << lines_merged.size() << "\n";
    std::cout << "Reduction from merging: " 
              << (lines.size() - lines_merged.size()) << " lines\n";
    
    // ========================================================================
    // Part 4: Line Filtering by Length and Angle
    // ========================================================================
    
    std::cout << "\n=== Line Filtering ===\n";
    
    // Filter lines by length
    std::vector<cv::Vec4f> long_lines;
    float min_length = 50.0f;
    
    for (const auto& line : lines) {
        float length = std::sqrt(
            std::pow(line[2] - line[0], 2) + 
            std::pow(line[3] - line[1], 2)
        );
        if (length >= min_length) {
            long_lines.push_back(line);
        }
    }
    
    std::cout << "Lines longer than " << min_length << " pixels: " 
              << long_lines.size() << "\n";
    
    // Filter lines by angle (e.g., near-horizontal lines)
    std::vector<cv::Vec4f> horizontal_lines;
    float max_angle = 15.0f;  // degrees from horizontal
    
    for (const auto& line : lines) {
        float angle = std::abs(std::atan2(
            line[3] - line[1], 
            line[2] - line[0]
        ) * 180.0f / CV_PI);
        
        if (angle < max_angle || angle > (180.0f - max_angle)) {
            horizontal_lines.push_back(line);
        }
    }
    
    std::cout << "Horizontal lines (±" << max_angle << "°): " 
              << horizontal_lines.size() << "\n";
    
    // ========================================================================
    // Part 5: Visualization
    // ========================================================================
    
    std::cout << "\n=== Visualization ===\n";
    
    // Visualize all lines
    cv::Mat image_lines;
    cv::cvtColor(image, image_lines, cv::COLOR_GRAY2BGR);
    fld->drawSegments(image_lines, lines);
    
    // Visualize merged lines
    cv::Mat image_merged;
    cv::cvtColor(image, image_merged, cv::COLOR_GRAY2BGR);
    fld_merge->drawSegments(image_merged, lines_merged);
    
    // Visualize filtered lines (color-coded)
    cv::Mat image_filtered;
    cv::cvtColor(image, image_filtered, cv::COLOR_GRAY2BGR);
    
    // Draw long lines in green
    for (const auto& line : long_lines) {
        cv::Point p1(static_cast<int>(line[0]), static_cast<int>(line[1]));
        cv::Point p2(static_cast<int>(line[2]), static_cast<int>(line[3]));
        cv::line(image_filtered, p1, p2, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    }
    
    // Draw horizontal lines in red
    for (const auto& line : horizontal_lines) {
        cv::Point p1(static_cast<int>(line[0]), static_cast<int>(line[1]));
        cv::Point p2(static_cast<int>(line[2]), static_cast<int>(line[3]));
        cv::line(image_filtered, p1, p2, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    }
    
    std::cout << "Visualization created. Press any key to close windows.\n";
    
    // Display
    cv::imshow("Original Image", image);
    cv::imshow("FLD - All Lines", image_lines);
    cv::imshow("FLD - Merged Lines", image_merged);
    cv::imshow("Filtered Lines (Green=long, Red=horizontal)", image_filtered);
    cv::waitKey(0);
    
    // Save output
    cv::imwrite("fld_all_lines.png", image_lines);
    cv::imwrite("fld_merged_lines.png", image_merged);
    cv::imwrite("fld_filtered_lines.png", image_filtered);
    std::cout << "Outputs saved.\n";
    
    return 0;
}

/*
 * Expected Output:
 * ================
 * Image size: 640x480
 * 
 * === FLD Detection (Default Parameters) ===
 * Detected lines: 200-400 (depends on image)
 * Detection time: 10-25 ms (typically faster than LSD)
 * Detection rate: 40-100 fps
 * 
 * First 5 detected lines:
 *   Line 0: (100,50) -> (200,50), length=100, angle=0°
 *   ...
 * 
 * === FLD Detection (Custom Parameters) ===
 * Detected lines (custom): 150-300
 * Detection time: 12-28 ms
 * 
 * === FLD Detection (With Merging) ===
 * Detected lines (merged): 100-200
 * Reduction from merging: 50-150 lines
 * 
 * === Line Filtering ===
 * Lines longer than 50 pixels: 50-100
 * Horizontal lines (±15°): 30-60
 * 
 * === Visualization ===
 * Visualization created. Press any key to close windows.
 * Outputs saved.
 * 
 * Notes:
 * ======
 * - FLD is typically 2-3x faster than LSD
 * - Output format: cv::Vec4f = [x1, y1, x2, y2]
 * - FLD is more sensitive to edges (uses Canny internally)
 * 
 * - Parameter tuning:
 *   * length_threshold: Adjust to filter short lines
 *   * distance_threshold: Controls segment merging (1.41 ≈ sqrt(2))
 *   * canny_th1, canny_th2: Edge detection sensitivity
 *   * do_merge: Reduces line count by merging collinear segments
 * 
 * - Advantages over LSD:
 *   * Faster execution (important for real-time SLAM)
 *   * Simpler API (no KeyLine struct, just Vec4f)
 *   * Good for structured environments (buildings, rooms)
 * 
 * - Disadvantages:
 *   * No built-in descriptor (need to compute separately)
 *   * May produce more fragmented lines without merging
 *   * Less robust in textured regions
 * 
 * - For ZenSLAM integration:
 *   * Can replace LSD for faster detection
 *   * Need to add descriptor computation step
 *   * Useful for real-time applications
 *   * vcpkg-friendly (part of opencv[contrib])
 */
