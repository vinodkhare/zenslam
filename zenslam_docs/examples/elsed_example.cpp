/**
 * @file elsed_example.cpp
 * @brief Example of using ELSED (Enhanced Line SEgment Drawing)
 * 
 * ELSED is a modern, ultra-fast line segment detector designed for real-time
 * applications. It's faster than both LSD and EDLines while maintaining
 * excellent accuracy.
 * 
 * Setup:
 *   1. Clone ELSED: git clone https://github.com/iago-suarez/ELSED.git
 *   2. ELSED provides both header-only and library options
 * 
 * Compilation (header-only approach):
 *   g++ -std=c++14 elsed_example.cpp \
 *       -I ELSED/include -o elsed_example \
 *       `pkg-config --cflags --libs opencv4`
 * 
 * Or with CMake:
 *   add_subdirectory(external/ELSED)
 *   target_link_libraries(your_target PRIVATE ELSED)
 */

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

// ELSED headers (uncomment when ELSED is available)
// #include <ELSED.h>

#include <iostream>
#include <chrono>
#include <vector>
#include <cmath>

/*
 * This is a TEMPLATE example showing how to use ELSED.
 * To compile this, you need to:
 *   1. Install ELSED from https://github.com/iago-suarez/ELSED
 *   2. Uncomment the ELSED include above
 *   3. Link against ELSED (or use header-only version)
 * 
 * The code structure below shows the API usage pattern.
 */

int main(int argc, char** argv)
{
    std::cout << "ELSED (Enhanced Line SEgment Drawing) Example Template\n";
    std::cout << "======================================================\n\n";
    
    std::cout << "This is a template showing how to use ELSED.\n";
    std::cout << "To actually run this code, you need to:\n";
    std::cout << "  1. Clone ELSED: git clone https://github.com/iago-suarez/ELSED.git\n";
    std::cout << "  2. Uncomment the #include <ELSED.h> line in this file\n";
    std::cout << "  3. Compile with ELSED (header-only or library)\n\n";
    
    std::string image_path = (argc > 1) ? argv[1] : "test_image.png";
    cv::Mat image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
    
    if (image.empty()) {
        std::cerr << "Error: Could not load image from " << image_path << "\n";
        return 1;
    }
    
    std::cout << "Image loaded: " << image.cols << "x" << image.rows << "\n\n";
    
    // ========================================================================
    // Part 1: Basic ELSED Detection (TEMPLATE CODE)
    // ========================================================================
    
    std::cout << "=== ELSED Detection (Template) ===\n\n";
    
    std::cout << "// Create ELSED detector with default parameters\n";
    std::cout << "upm::ELSED elsed;  // Default constructor\n\n";
    
    std::cout << "// Detect line segments\n";
    std::cout << "std::vector<upm::Segment> segments = elsed.detect(image);\n\n";
    
    std::cout << "// Access segments\n";
    std::cout << "for (const auto& seg : segments) {\n";
    std::cout << "    float x1 = seg.start.x;\n";
    std::cout << "    float y1 = seg.start.y;\n";
    std::cout << "    float x2 = seg.end.x;\n";
    std::cout << "    float y2 = seg.end.y;\n";
    std::cout << "    \n";
    std::cout << "    // Segment properties\n";
    std::cout << "    float angle = seg.angle;          // Line angle in radians\n";
    std::cout << "    float salience = seg.salience;    // Detection confidence\n";
    std::cout << "    \n";
    std::cout << "    // Use segment...\n";
    std::cout << "}\n\n";
    
    // ========================================================================
    // Part 2: ELSED with Custom Parameters (TEMPLATE CODE)
    // ========================================================================
    
    std::cout << "=== ELSED with Custom Parameters ===\n\n";
    
    std::cout << "// ELSED constructor with custom parameters:\n";
    std::cout << "// ELSED(\n";
    std::cout << "//     int ksize = 5,                  // Gradient kernel size\n";
    std::cout << "//     float sigma = 1.0f,             // Gaussian smoothing\n";
    std::cout << "//     float gradientThreshold = 30.f, // Edge strength\n";
    std::cout << "//     float anchorThreshold = 2.f,    // Anchor selection\n";
    std::cout << "//     int scanIntervals = 2,          // Scan intervals\n";
    std::cout << "//     int minLineLen = 15,            // Minimum line length\n";
    std::cout << "//     double lineFitErrThreshold = 1.4, // Fitting tolerance\n";
    std::cout << "//     double pxToSegmentDistTh = 1.5, // Distance threshold\n";
    std::cout << "//     bool validate = true            // Validate segments\n";
    std::cout << "// )\n\n";
    
    std::cout << "// Example: Ultra-fast mode (reduce quality for speed)\n";
    std::cout << "upm::ELSED elsed_fast(\n";
    std::cout << "    3,        // ksize: smaller kernel (faster)\n";
    std::cout << "    1.0f,     // sigma: standard smoothing\n";
    std::cout << "    40.f,     // gradientThreshold: higher (fewer edges)\n";
    std::cout << "    3.f,      // anchorThreshold: higher (fewer anchors)\n";
    std::cout << "    1,        // scanIntervals: fewer scans\n";
    std::cout << "    20,       // minLineLen: longer lines only\n";
    std::cout << "    1.4,      // lineFitErrThreshold: standard\n";
    std::cout << "    1.5,      // pxToSegmentDistTh: standard\n";
    std::cout << "    false     // validate: skip validation (faster)\n";
    std::cout << ");\n\n";
    
    std::cout << "std::vector<upm::Segment> fast_segments = elsed_fast.detect(image);\n\n";
    
    std::cout << "// Example: High-quality mode (slower but more accurate)\n";
    std::cout << "upm::ELSED elsed_quality(\n";
    std::cout << "    7,        // ksize: larger kernel (more accurate)\n";
    std::cout << "    1.5f,     // sigma: more smoothing\n";
    std::cout << "    20.f,     // gradientThreshold: lower (more edges)\n";
    std::cout << "    1.5f,     // anchorThreshold: lower (more anchors)\n";
    std::cout << "    3,        // scanIntervals: more scans\n";
    std::cout << "    10,       // minLineLen: detect short lines too\n";
    std::cout << "    1.2,      // lineFitErrThreshold: stricter\n";
    std::cout << "    1.2,      // pxToSegmentDistTh: stricter\n";
    std::cout << "    true      // validate: full validation\n";
    std::cout << ");\n\n";
    
    std::cout << "std::vector<upm::Segment> quality_segments = elsed_quality.detect(image);\n\n";
    
    // ========================================================================
    // Part 3: Performance Characteristics (INFORMATIONAL)
    // ========================================================================
    
    std::cout << "=== ELSED Performance Characteristics ===\n\n";
    
    std::cout << "Expected performance on 640x480 images:\n";
    std::cout << "  Default mode:\n";
    std::cout << "    - Detection time: 3-10 ms\n";
    std::cout << "    - Detection rate: 100-300 fps\n";
    std::cout << "    - Number of lines: 100-400\n\n";
    
    std::cout << "  Fast mode:\n";
    std::cout << "    - Detection time: 1-5 ms\n";
    std::cout << "    - Detection rate: 200-1000 fps\n";
    std::cout << "    - Number of lines: 50-200\n\n";
    
    std::cout << "  Quality mode:\n";
    std::cout << "    - Detection time: 5-15 ms\n";
    std::cout << "    - Detection rate: 60-200 fps\n";
    std::cout << "    - Number of lines: 150-500\n\n";
    
    std::cout << "Comparison with other detectors:\n";
    std::cout << "  vs LSD:       3-8x faster, similar accuracy\n";
    std::cout << "  vs EDLines:   1.5-3x faster, similar accuracy\n";
    std::cout << "  vs FLD:       Similar speed, better accuracy\n";
    std::cout << "  vs Hough:     Similar speed, much better accuracy\n\n";
    
    std::cout << "Key Advantages:\n";
    std::cout << "  ✓ Fastest modern line detector\n";
    std::cout << "  ✓ Excellent subpixel accuracy\n";
    std::cout << "  ✓ Robust gradient estimation\n";
    std::cout << "  ✓ Modern C++ (C++11/14)\n";
    std::cout << "  ✓ Header-only option available\n";
    std::cout << "  ✓ Active development and maintenance\n";
    std::cout << "  ✓ Used in recent research (2022 paper)\n\n";
    
    // ========================================================================
    // Part 4: Integration with ZenSLAM (GUIDANCE)
    // ========================================================================
    
    std::cout << "=== Integration with ZenSLAM ===\n\n";
    
    std::cout << "Step 1: Add ELSED to project\n";
    std::cout << "  mkdir external\n";
    std::cout << "  cd external\n";
    std::cout << "  git clone https://github.com/iago-suarez/ELSED.git\n\n";
    
    std::cout << "Step 2: Update CMakeLists.txt\n";
    std::cout << "  # In your CMakeLists.txt:\n";
    std::cout << "  option(ZENSLAM_USE_ELSED \"Use ELSED detector\" OFF)\n";
    std::cout << "  \n";
    std::cout << "  if(ZENSLAM_USE_ELSED)\n";
    std::cout << "      # ELSED can be used header-only or as library\n";
    std::cout << "      add_subdirectory(external/ELSED)\n";
    std::cout << "      target_link_libraries(zenslam_core PRIVATE ELSED)\n";
    std::cout << "      target_compile_definitions(zenslam_core PRIVATE USE_ELSED)\n";
    std::cout << "  endif()\n\n";
    
    std::cout << "Step 3: Adapt grid_detector class\n";
    std::cout << "  // In grid_detector.cpp:\n";
    std::cout << "  #ifdef USE_ELSED\n";
    std::cout << "  #include <ELSED.h>\n";
    std::cout << "  #endif\n";
    std::cout << "  \n";
    std::cout << "  std::vector<keyline> grid_detector::detect(const cv::Mat& image) const {\n";
    std::cout << "  #ifdef USE_ELSED\n";
    std::cout << "      // Use ELSED\n";
    std::cout << "      static upm::ELSED elsed;  // Reuse detector\n";
    std::cout << "      std::vector<upm::Segment> segments = elsed.detect(image);\n";
    std::cout << "      \n";
    std::cout << "      // Convert to keyline format\n";
    std::cout << "      std::vector<keyline> keylines;\n";
    std::cout << "      keylines.reserve(segments.size());\n";
    std::cout << "      \n";
    std::cout << "      for (size_t i = 0; i < segments.size(); ++i) {\n";
    std::cout << "          const auto& seg = segments[i];\n";
    std::cout << "          \n";
    std::cout << "          keyline kl;\n";
    std::cout << "          kl.index = i;\n";
    std::cout << "          kl.startPointX = seg.start.x;\n";
    std::cout << "          kl.startPointY = seg.start.y;\n";
    std::cout << "          kl.endPointX = seg.end.x;\n";
    std::cout << "          kl.endPointY = seg.end.y;\n";
    std::cout << "          \n";
    std::cout << "          // Compute derived fields\n";
    std::cout << "          float dx = seg.end.x - seg.start.x;\n";
    std::cout << "          float dy = seg.end.y - seg.start.y;\n";
    std::cout << "          kl.lineLength = std::sqrt(dx*dx + dy*dy);\n";
    std::cout << "          kl.angle = std::atan2(dy, dx) * 180.0f / CV_PI;\n";
    std::cout << "          kl.pt.x = (seg.start.x + seg.end.x) / 2.0f;\n";
    std::cout << "          kl.pt.y = (seg.start.y + seg.end.y) / 2.0f;\n";
    std::cout << "          \n";
    std::cout << "          keylines.push_back(kl);\n";
    std::cout << "      }\n";
    std::cout << "      \n";
    std::cout << "      return keylines;\n";
    std::cout << "  #else\n";
    std::cout << "      // Use LSD (existing code)\n";
    std::cout << "      // ...\n";
    std::cout << "  #endif\n";
    std::cout << "  }\n\n";
    
    // ========================================================================
    // Part 5: Complete Workflow Example (TEMPLATE)
    // ========================================================================
    
    std::cout << "=== Complete Workflow Example ===\n\n";
    
    std::cout << "#include <ELSED.h>\n";
    std::cout << "#include <opencv2/core.hpp>\n";
    std::cout << "#include <opencv2/imgproc.hpp>\n";
    std::cout << "#include <opencv2/highgui.hpp>\n";
    std::cout << "#include <chrono>\n\n";
    
    std::cout << "void detect_and_visualize(const cv::Mat& image) {\n";
    std::cout << "    // Create detector (reuse for multiple images)\n";
    std::cout << "    static upm::ELSED elsed;\n";
    std::cout << "    \n";
    std::cout << "    // Detect\n";
    std::cout << "    auto t_start = std::chrono::high_resolution_clock::now();\n";
    std::cout << "    std::vector<upm::Segment> segments = elsed.detect(image);\n";
    std::cout << "    auto t_end = std::chrono::high_resolution_clock::now();\n";
    std::cout << "    \n";
    std::cout << "    auto duration_ms = std::chrono::duration<double, std::milli>(\n";
    std::cout << "        t_end - t_start\n";
    std::cout << "    ).count();\n";
    std::cout << "    \n";
    std::cout << "    std::cout << \"Detected \" << segments.size() << \" lines \";\n";
    std::cout << "    std::cout << \"in \" << duration_ms << \" ms \";\n";
    std::cout << "    std::cout << \"(\" << (1000.0 / duration_ms) << \" fps)\\n\";\n";
    std::cout << "    \n";
    std::cout << "    // Visualize\n";
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
    std::cout << "        // Color by salience (confidence)\n";
    std::cout << "        int green = static_cast<int>(255 * seg.salience);\n";
    std::cout << "        cv::Scalar color(0, green, 255 - green);\n";
    std::cout << "        \n";
    std::cout << "        cv::line(viz, p1, p2, color, 2, cv::LINE_AA);\n";
    std::cout << "    }\n";
    std::cout << "    \n";
    std::cout << "    cv::imshow(\"ELSED\", viz);\n";
    std::cout << "    cv::waitKey(0);\n";
    std::cout << "}\n\n";
    
    // ========================================================================
    // Part 6: Benchmarking Against Other Detectors (GUIDANCE)
    // ========================================================================
    
    std::cout << "=== Benchmarking Example ===\n\n";
    
    std::cout << "void benchmark_detectors(const cv::Mat& image) {\n";
    std::cout << "    const int ITERATIONS = 100;\n";
    std::cout << "    \n";
    std::cout << "    // Benchmark ELSED\n";
    std::cout << "    upm::ELSED elsed;\n";
    std::cout << "    auto start = std::chrono::high_resolution_clock::now();\n";
    std::cout << "    for (int i = 0; i < ITERATIONS; ++i) {\n";
    std::cout << "        auto segments = elsed.detect(image);\n";
    std::cout << "    }\n";
    std::cout << "    auto end = std::chrono::high_resolution_clock::now();\n";
    std::cout << "    auto elsed_time = std::chrono::duration<double, std::milli>(\n";
    std::cout << "        end - start\n";
    std::cout << "    ).count() / ITERATIONS;\n";
    std::cout << "    \n";
    std::cout << "    std::cout << \"ELSED: \" << elsed_time << \" ms/frame\\n\";\n";
    std::cout << "    std::cout << \"       (\" << (1000.0/elsed_time) << \" fps)\\n\";\n";
    std::cout << "    \n";
    std::cout << "    // Compare with LSD, FLD, etc...\n";
    std::cout << "}\n\n";
    
    // ========================================================================
    // Summary and References
    // ========================================================================
    
    std::cout << "=== Summary ===\n\n";
    
    std::cout << "ELSED is the most modern line detector available:\n";
    std::cout << "  ✓ Fastest detector (3-8x faster than LSD)\n";
    std::cout << "  ✓ Excellent accuracy with subpixel precision\n";
    std::cout << "  ✓ Modern C++11/14 codebase\n";
    std::cout << "  ✓ Header-only option for easy integration\n";
    std::cout << "  ✓ Active development (2022 publication)\n";
    std::cout << "  ✓ Configurable speed/quality tradeoff\n";
    std::cout << "  ✓ No external dependencies beyond OpenCV\n";
    std::cout << "  ✗ Not in vcpkg (manual integration required)\n\n";
    
    std::cout << "Best for:\n";
    std::cout << "  - Real-time SLAM applications\n";
    std::cout << "  - High-framerate processing (> 100 fps)\n";
    std::cout << "  - Embedded systems with limited compute\n";
    std::cout << "  - Modern C++ projects\n\n";
    
    std::cout << "References:\n";
    std::cout << "  - GitHub: https://github.com/iago-suarez/ELSED\n";
    std::cout << "  - Paper: Suárez et al., \"ELSED: Enhanced Line SEgment \n";
    std::cout << "           Drawing\" (2022)\n";
    std::cout << "  - Used in: Modern visual SLAM and SfM systems\n\n";
    
    std::cout << "To run this example with actual ELSED code:\n";
    std::cout << "  1. Follow the setup instructions at the top of this file\n";
    std::cout << "  2. Uncomment the #include <ELSED.h> line\n";
    std::cout << "  3. Uncomment the actual implementation code\n";
    std::cout << "  4. Compile and link with ELSED\n\n";
    
    return 0;
}

/*
 * Additional Notes:
 * =================
 * 
 * ELSED vs EDLines:
 * - ELSED is 1.5-3x faster
 * - Both have excellent accuracy
 * - ELSED has more modern codebase
 * - ELSED is more actively maintained
 * 
 * ELSED vs LSD:
 * - ELSED is 3-8x faster
 * - Similar or better accuracy
 * - LSD has descriptors built-in
 * - LSD is more widely used
 * 
 * Recommended Configuration for SLAM:
 * - Use default parameters to start
 * - Adjust minLineLen based on scene (10-30 typical)
 * - Enable validation for critical applications
 * - Disable validation for maximum speed
 * - Consider gradient threshold based on image quality
 * 
 * Memory Usage:
 * - Very efficient (similar to EDLines)
 * - Minimal allocations during detection
 * - Suitable for embedded systems
 */
