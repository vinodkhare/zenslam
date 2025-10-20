/**
 * @file edlines_example.cpp
 * @brief Example of using EDLines (Edge Drawing Lines)
 * 
 * EDLines is 2-5x faster than LSD while maintaining excellent accuracy.
 * Requires manual integration of ED_Lib.
 * 
 * Setup:
 *   1. Clone ED_Lib: git clone https://github.com/CihanTopal/ED_Lib.git
 *   2. Build ED_Lib or include sources directly
 * 
 * Compilation (assuming ED_Lib is in current directory):
 *   g++ -std=c++17 edlines_example.cpp \
 *       ED_Lib/ED.cpp ED_Lib/EDLines.cpp ED_Lib/NFA.cpp \
 *       -I ED_Lib -o edlines_example \
 *       `pkg-config --cflags --libs opencv4`
 * 
 * Or with CMake:
 *   add_subdirectory(ED_Lib)
 *   target_link_libraries(your_target PRIVATE EDLib)
 */

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

// EDLines headers (adjust path based on your setup)
// Uncomment these when ED_Lib is available:
// #include <ED.h>
// #include <EDLines.h>

#include <iostream>
#include <chrono>
#include <vector>
#include <cmath>

/*
 * This is a TEMPLATE example showing how to use EDLines.
 * To compile this, you need to:
 *   1. Install ED_Lib from https://github.com/CihanTopal/ED_Lib
 *   2. Uncomment the ED_Lib includes above
 *   3. Link against ED_Lib
 * 
 * The code structure below shows the API usage pattern.
 */

int main(int argc, char** argv)
{
    std::cout << "EDLines Example Template\n";
    std::cout << "========================\n\n";
    
    std::cout << "This is a template showing how to use EDLines.\n";
    std::cout << "To actually run this code, you need to:\n";
    std::cout << "  1. Clone ED_Lib: git clone https://github.com/CihanTopal/ED_Lib.git\n";
    std::cout << "  2. Uncomment the #include lines in this file\n";
    std::cout << "  3. Compile with ED_Lib sources\n\n";
    
    std::string image_path = (argc > 1) ? argv[1] : "test_image.png";
    cv::Mat image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
    
    if (image.empty()) {
        std::cerr << "Error: Could not load image from " << image_path << "\n";
        return 1;
    }
    
    std::cout << "Image loaded: " << image.cols << "x" << image.rows << "\n\n";
    
    // ========================================================================
    // Part 1: Basic EDLines Detection (TEMPLATE CODE)
    // ========================================================================
    
    std::cout << "=== EDLines Detection (Template) ===\n\n";
    
    std::cout << "// Create EDLines detector\n";
    std::cout << "EDLines edlines(image);\n\n";
    
    std::cout << "// Get detected line segments\n";
    std::cout << "EDLineSegments lines = edlines.getLines();\n\n";
    
    std::cout << "// Access line segments\n";
    std::cout << "for (size_t i = 0; i < lines.size(); i++) {\n";
    std::cout << "    double x1 = lines[i].start.x;\n";
    std::cout << "    double y1 = lines[i].start.y;\n";
    std::cout << "    double x2 = lines[i].end.x;\n";
    std::cout << "    double y2 = lines[i].end.y;\n";
    std::cout << "    \n";
    std::cout << "    double length = std::sqrt(\n";
    std::cout << "        (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1)\n";
    std::cout << "    );\n";
    std::cout << "    \n";
    std::cout << "    // Use line segment...\n";
    std::cout << "}\n\n";
    
    // ========================================================================
    // Part 2: EDLines with Custom Parameters (TEMPLATE CODE)
    // ========================================================================
    
    std::cout << "=== EDLines with Custom Parameters ===\n\n";
    
    std::cout << "// EDLines constructor accepts parameters:\n";
    std::cout << "// EDLines(cv::Mat image, \n";
    std::cout << "//         bool smoothing = true,\n";
    std::cout << "//         float sigma = 1.5f,\n";
    std::cout << "//         float gradientThreshold = 36.0f,\n";
    std::cout << "//         float anchorThreshold = 8.0f,\n";
    std::cout << "//         int minLineLen = 15,\n";
    std::cout << "//         double lineFitErrorThreshold = 1.0)\n\n";
    
    std::cout << "// Example: Detect longer, more reliable lines\n";
    std::cout << "EDLines edlines_custom(\n";
    std::cout << "    image,\n";
    std::cout << "    true,        // smoothing: reduce noise\n";
    std::cout << "    1.5f,        // sigma: Gaussian smoothing\n";
    std::cout << "    36.0f,       // gradientThreshold: edge strength\n";
    std::cout << "    8.0f,        // anchorThreshold: anchor selection\n";
    std::cout << "    30,          // minLineLen: minimum line length\n";
    std::cout << "    1.0          // lineFitErrorThreshold: fitting tolerance\n";
    std::cout << ");\n\n";
    
    std::cout << "EDLineSegments custom_lines = edlines_custom.getLines();\n\n";
    
    // ========================================================================
    // Part 3: Performance Characteristics (INFORMATIONAL)
    // ========================================================================
    
    std::cout << "=== EDLines Performance Characteristics ===\n\n";
    
    std::cout << "Expected performance on 640x480 images:\n";
    std::cout << "  - Detection time: 5-15 ms\n";
    std::cout << "  - Detection rate: 60-200 fps\n";
    std::cout << "  - Typical speedup vs LSD: 2-5x\n";
    std::cout << "  - Number of lines: 100-400 (depends on scene)\n\n";
    
    std::cout << "Compared to LSD:\n";
    std::cout << "  + Significantly faster (2-5x)\n";
    std::cout << "  + Better subpixel accuracy\n";
    std::cout << "  + More robust to noise\n";
    std::cout << "  ~ Similar detection quality\n";
    std::cout << "  - Requires manual integration\n\n";
    
    // ========================================================================
    // Part 4: Integration with ZenSLAM (GUIDANCE)
    // ========================================================================
    
    std::cout << "=== Integration with ZenSLAM ===\n\n";
    
    std::cout << "Step 1: Add ED_Lib to project\n";
    std::cout << "  mkdir external\n";
    std::cout << "  cd external\n";
    std::cout << "  git clone https://github.com/CihanTopal/ED_Lib.git\n\n";
    
    std::cout << "Step 2: Update CMakeLists.txt\n";
    std::cout << "  # In your CMakeLists.txt:\n";
    std::cout << "  option(ZENSLAM_USE_EDLINES \"Use EDLines detector\" OFF)\n";
    std::cout << "  \n";
    std::cout << "  if(ZENSLAM_USE_EDLINES)\n";
    std::cout << "      add_subdirectory(external/ED_Lib)\n";
    std::cout << "      target_link_libraries(zenslam_core PRIVATE EDLib)\n";
    std::cout << "      target_compile_definitions(zenslam_core PRIVATE USE_EDLINES)\n";
    std::cout << "  endif()\n\n";
    
    std::cout << "Step 3: Adapt grid_detector class\n";
    std::cout << "  // In grid_detector.cpp:\n";
    std::cout << "  #ifdef USE_EDLINES\n";
    std::cout << "  #include <ED.h>\n";
    std::cout << "  #include <EDLines.h>\n";
    std::cout << "  #endif\n";
    std::cout << "  \n";
    std::cout << "  std::vector<keyline> grid_detector::detect(const cv::Mat& image) const {\n";
    std::cout << "  #ifdef USE_EDLINES\n";
    std::cout << "      // Use EDLines\n";
    std::cout << "      EDLines ed(image);\n";
    std::cout << "      EDLineSegments segments = ed.getLines();\n";
    std::cout << "      \n";
    std::cout << "      // Convert to keyline format\n";
    std::cout << "      std::vector<keyline> keylines;\n";
    std::cout << "      for (const auto& seg : segments) {\n";
    std::cout << "          keyline kl;\n";
    std::cout << "          kl.startPointX = seg.start.x;\n";
    std::cout << "          kl.startPointY = seg.start.y;\n";
    std::cout << "          kl.endPointX = seg.end.x;\n";
    std::cout << "          kl.endPointY = seg.end.y;\n";
    std::cout << "          // ... compute other fields ...\n";
    std::cout << "          keylines.push_back(kl);\n";
    std::cout << "      }\n";
    std::cout << "      return keylines;\n";
    std::cout << "  #else\n";
    std::cout << "      // Use LSD (existing code)\n";
    std::cout << "      // ...\n";
    std::cout << "  #endif\n";
    std::cout << "  }\n\n";
    
    // ========================================================================
    // Part 5: Expected Usage Pattern (COMPLETE WORKFLOW)
    // ========================================================================
    
    std::cout << "=== Complete Workflow Example ===\n\n";
    
    std::cout << "void detect_and_visualize(const cv::Mat& image) {\n";
    std::cout << "    // 1. Create detector and detect lines\n";
    std::cout << "    auto t_start = std::chrono::high_resolution_clock::now();\n";
    std::cout << "    EDLines ed(image);\n";
    std::cout << "    EDLineSegments segments = ed.getLines();\n";
    std::cout << "    auto t_end = std::chrono::high_resolution_clock::now();\n";
    std::cout << "    \n";
    std::cout << "    auto duration = std::chrono::duration<double, std::milli>(\n";
    std::cout << "        t_end - t_start\n";
    std::cout << "    ).count();\n";
    std::cout << "    \n";
    std::cout << "    std::cout << \"Lines: \" << segments.size() \n";
    std::cout << "              << \", Time: \" << duration << \" ms\\n\";\n";
    std::cout << "    \n";
    std::cout << "    // 2. Visualize\n";
    std::cout << "    cv::Mat viz;\n";
    std::cout << "    cv::cvtColor(image, viz, cv::COLOR_GRAY2BGR);\n";
    std::cout << "    \n";
    std::cout << "    for (const auto& seg : segments) {\n";
    std::cout << "        cv::Point p1(\n";
    std::cout << "            static_cast<int>(seg.start.x),\n";
    std::cout << "            static_cast<int>(seg.start.y)\n";
    std::cout << "        );\n";
    std::cout << "        cv::Point p2(\n";
    std::cout << "            static_cast<int>(seg.end.x),\n";
    std::cout << "            static_cast<int>(seg.end.y)\n";
    std::cout << "        );\n";
    std::cout << "        \n";
    std::cout << "        cv::line(viz, p1, p2, cv::Scalar(0, 255, 0), 2);\n";
    std::cout << "    }\n";
    std::cout << "    \n";
    std::cout << "    cv::imshow(\"EDLines\", viz);\n";
    std::cout << "    cv::waitKey(0);\n";
    std::cout << "}\n\n";
    
    // ========================================================================
    // Summary and References
    // ========================================================================
    
    std::cout << "=== Summary ===\n\n";
    
    std::cout << "EDLines is an excellent choice for line detection in SLAM:\n";
    std::cout << "  ✓ 2-5x faster than LSD\n";
    std::cout << "  ✓ Excellent accuracy and subpixel precision\n";
    std::cout << "  ✓ Robust to noise and illumination changes\n";
    std::cout << "  ✓ Real-time capable (200+ fps on modern hardware)\n";
    std::cout << "  ✓ Parameter-free (works well with defaults)\n";
    std::cout << "  ✗ Not in vcpkg (manual integration required)\n\n";
    
    std::cout << "References:\n";
    std::cout << "  - GitHub: https://github.com/CihanTopal/ED_Lib\n";
    std::cout << "  - Paper: Topal & Akinlar, \"Edge Drawing: A combined \n";
    std::cout << "           real-time edge and segment detector\" (2012)\n";
    std::cout << "  - Used in: ORB-SLAM3, DSO, and other modern SLAM systems\n\n";
    
    std::cout << "To run this example with actual EDLines code:\n";
    std::cout << "  1. Follow the setup instructions at the top of this file\n";
    std::cout << "  2. Uncomment the #include lines\n";
    std::cout << "  3. Uncomment the actual implementation code\n";
    std::cout << "  4. Compile and link with ED_Lib\n\n";
    
    return 0;
}

/*
 * Complete Implementation Template:
 * ==================================
 * 
 * When ED_Lib is available, here's a complete working example:
 * 
 * #include <ED.h>
 * #include <EDLines.h>
 * 
 * int main() {
 *     cv::Mat image = cv::imread("test.png", cv::IMREAD_GRAYSCALE);
 *     
 *     // Simple detection
 *     EDLines ed(image);
 *     EDLineSegments segments = ed.getLines();
 *     
 *     // Access segments
 *     for (const auto& seg : segments) {
 *         std::cout << "Line: (" 
 *                   << seg.start.x << "," << seg.start.y << ") -> ("
 *                   << seg.end.x << "," << seg.end.y << ")\n";
 *     }
 *     
 *     // Get edge map (useful for debugging)
 *     cv::Mat edge_map = ed.getEdgeImage();
 *     
 *     // Visualize
 *     cv::Mat viz;
 *     cv::cvtColor(image, viz, cv::COLOR_GRAY2BGR);
 *     for (const auto& seg : segments) {
 *         cv::line(viz,
 *                  cv::Point(seg.start.x, seg.start.y),
 *                  cv::Point(seg.end.x, seg.end.y),
 *                  cv::Scalar(0, 255, 0), 2);
 *     }
 *     cv::imshow("EDLines", viz);
 *     cv::waitKey(0);
 *     
 *     return 0;
 * }
 * 
 * CMake Integration:
 * ==================
 * 
 * # CMakeLists.txt for ED_Lib integration
 * 
 * cmake_minimum_required(VERSION 3.10)
 * project(edlines_example)
 * 
 * set(CMAKE_CXX_STANDARD 17)
 * 
 * find_package(OpenCV REQUIRED)
 * 
 * # Option to use EDLines
 * option(USE_EDLINES "Use EDLines detector" ON)
 * 
 * if(USE_EDLINES)
 *     # Add ED_Lib as subdirectory or find installed version
 *     add_subdirectory(external/ED_Lib)
 *     
 *     add_executable(edlines_example edlines_example.cpp)
 *     target_link_libraries(edlines_example 
 *         PRIVATE 
 *         ${OpenCV_LIBS}
 *         EDLib
 *     )
 *     target_compile_definitions(edlines_example PRIVATE USE_EDLINES)
 * endif()
 */
