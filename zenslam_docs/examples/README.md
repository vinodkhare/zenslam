# Line Segment Detection Library Examples

This directory contains working code examples demonstrating various line segment detection libraries that can be used with C++.

## Overview

These examples accompany the [Line Segment Libraries Research](../line_segment_libraries_research.md) document and provide practical, compilable code samples for each library.

## Available Examples

### 1. `lsd_example.cpp` - OpenCV LSD Detector
**Status**: ✅ Ready to compile (requires opencv[contrib])

The Line Segment Detector from OpenCV's contrib module. This is the **current solution used in ZenSLAM**.

**Features demonstrated**:
- Basic line detection with LSD
- Binary descriptor computation
- Detection with masking
- Visualization of results
- Performance benchmarking

**Compilation**:
```bash
# With pkg-config
g++ -std=c++17 lsd_example.cpp -o lsd_example \
    `pkg-config --cflags --libs opencv4`

# With vcpkg
g++ -std=c++17 lsd_example.cpp -o lsd_example \
    -I${VCPKG_ROOT}/installed/x64-linux/include \
    -L${VCPKG_ROOT}/installed/x64-linux/lib \
    -lopencv_core -lopencv_imgproc -lopencv_imgcodecs \
    -lopencv_line_descriptor -lopencv_highgui
```

**Usage**:
```bash
./lsd_example test_image.png
```

---

### 2. `fld_example.cpp` - OpenCV Fast Line Detector
**Status**: ✅ Ready to compile (requires opencv[contrib])

OpenCV's Fast Line Detector from the ximgproc module. **Best vcpkg-friendly alternative** to LSD.

**Features demonstrated**:
- Default and custom parameter detection
- Line merging for cleaner results
- Line filtering by length and angle
- Multiple visualization modes
- Parameter sensitivity analysis

**Compilation**:
```bash
# With pkg-config
g++ -std=c++17 fld_example.cpp -o fld_example \
    `pkg-config --cflags --libs opencv4`

# With vcpkg
g++ -std=c++17 fld_example.cpp -o fld_example \
    -I${VCPKG_ROOT}/installed/x64-linux/include \
    -L${VCPKG_ROOT}/installed/x64-linux/lib \
    -lopencv_core -lopencv_imgproc -lopencv_imgcodecs \
    -lopencv_ximgproc -lopencv_highgui
```

**Usage**:
```bash
./fld_example test_image.png
```

---

### 3. `hough_example.cpp` - OpenCV Hough Transform
**Status**: ✅ Ready to compile (requires core opencv only)

Classical Hough Line Transform from OpenCV core. No contrib modules required.

**Features demonstrated**:
- Canny edge detection
- Standard Hough Transform (infinite lines)
- Probabilistic Hough Transform (segments)
- Parameter sensitivity testing
- Comparison visualization

**Compilation**:
```bash
# With pkg-config
g++ -std=c++17 hough_example.cpp -o hough_example \
    `pkg-config --cflags --libs opencv4`

# With vcpkg
g++ -std=c++17 hough_example.cpp -o hough_example \
    -I${VCPKG_ROOT}/installed/x64-linux/include \
    -L${VCPKG_ROOT}/installed/x64-linux/lib \
    -lopencv_core -lopencv_imgproc -lopencv_imgcodecs \
    -lopencv_highgui
```

**Usage**:
```bash
./hough_example test_image.png
```

---

### 4. `edlines_example.cpp` - EDLines Detector (Template)
**Status**: 📝 Template code (requires ED_Lib installation)

EDLines from the ED_Lib library. **Best performance alternative** (2-5x faster than LSD).

**Features demonstrated**:
- API usage patterns
- Custom parameter configuration
- Integration guidance for ZenSLAM
- Complete workflow examples

**Setup**:
```bash
# Clone ED_Lib
git clone https://github.com/CihanTopal/ED_Lib.git
```

**Compilation**:
```bash
# After ED_Lib is available
g++ -std=c++17 edlines_example.cpp \
    ED_Lib/ED.cpp ED_Lib/EDLines.cpp ED_Lib/NFA.cpp \
    -I ED_Lib -o edlines_example \
    `pkg-config --cflags --libs opencv4`
```

**Note**: This is a template showing the API. Uncomment the includes and implementation code once ED_Lib is installed.

---

### 5. `elsed_example.cpp` - ELSED Detector (Template)
**Status**: 📝 Template code (requires ELSED installation)

Enhanced Line SEgment Drawing. **Most modern option** with best speed/accuracy tradeoff.

**Features demonstrated**:
- API usage patterns
- Fast and quality mode configurations
- Performance characteristics
- Integration guidance for ZenSLAM
- Benchmarking examples

**Setup**:
```bash
# Clone ELSED
git clone https://github.com/iago-suarez/ELSED.git
```

**Compilation**:
```bash
# After ELSED is available (header-only option)
g++ -std=c++14 elsed_example.cpp \
    -I ELSED/include -o elsed_example \
    `pkg-config --cflags --libs opencv4`
```

**Note**: This is a template showing the API. Uncomment the includes and implementation code once ELSED is installed.

---

## Quick Comparison

| Example | Compile Ready | vcpkg | Speed | Accuracy | Best For |
|---------|---------------|-------|-------|----------|----------|
| **LSD** | ✅ | ✅ | Good | Excellent | Current solution |
| **FLD** | ✅ | ✅ | Excellent | Good | Fast + vcpkg |
| **Hough** | ✅ | ✅ | Good | Fair | Simple scenes |
| **EDLines** | 📝 | ❌ | Excellent | Excellent | Max performance |
| **ELSED** | 📝 | ❌ | Excellent | Excellent | Modern projects |

**Legend**:
- ✅ = Ready to compile
- 📝 = Template (needs library installation)
- ❌ = Not available in vcpkg
- Speed/Accuracy: Fair < Good < Excellent

---

## Building All Examples

### Prerequisites

**For LSD, FLD, and Hough examples** (vcpkg):
```bash
vcpkg install opencv[contrib]
```

**For EDLines example** (manual):
```bash
cd examples
git clone https://github.com/CihanTopal/ED_Lib.git
```

**For ELSED example** (manual):
```bash
cd examples
git clone https://github.com/iago-suarez/ELSED.git
```

### Build Script

Create a `build_examples.sh`:
```bash
#!/bin/bash
set -e

# Build OpenCV-based examples (ready to compile)
echo "Building LSD example..."
g++ -std=c++17 lsd_example.cpp -o lsd_example \
    `pkg-config --cflags --libs opencv4`

echo "Building FLD example..."
g++ -std=c++17 fld_example.cpp -o fld_example \
    `pkg-config --cflags --libs opencv4`

echo "Building Hough example..."
g++ -std=c++17 hough_example.cpp -o hough_example \
    `pkg-config --cflags --libs opencv4`

# EDLines (if available)
if [ -d "ED_Lib" ]; then
    echo "Building EDLines example..."
    g++ -std=c++17 edlines_example.cpp \
        ED_Lib/ED.cpp ED_Lib/EDLines.cpp ED_Lib/NFA.cpp \
        -I ED_Lib -o edlines_example \
        `pkg-config --cflags --libs opencv4`
else
    echo "Skipping EDLines (ED_Lib not found)"
fi

# ELSED (if available)
if [ -d "ELSED" ]; then
    echo "Building ELSED example..."
    g++ -std=c++14 elsed_example.cpp \
        -I ELSED/include -o elsed_example \
        `pkg-config --cflags --libs opencv4`
else
    echo "Skipping ELSED (ELSED not found)"
fi

echo "Build complete!"
```

---

## Running the Examples

All examples accept an optional image path argument:

```bash
./lsd_example [image_path]
./fld_example [image_path]
./hough_example [image_path]
./edlines_example [image_path]
./elsed_example [image_path]
```

If no image is provided, they look for `test_image.png` in the current directory.

### Creating a Test Image

You can use any grayscale image, or create one:

```bash
# Download a sample image
wget https://upload.wikimedia.org/wikipedia/commons/thumb/2/2c/Rotating_earth_%28large%29.gif/240px-Rotating_earth_%28large%29.gif -O test.gif
convert test.gif[0] -resize 640x480 -colorspace gray test_image.png

# Or use an existing image from your dataset
cp /path/to/your/stereo/dataset/image.png test_image.png
```

---

## Performance Testing

Each example includes timing information. To benchmark:

```bash
# Run each detector on the same image
./lsd_example test_image.png 2>&1 | grep "Detection rate"
./fld_example test_image.png 2>&1 | grep "Detection rate"
./hough_example test_image.png 2>&1 | grep "Detection rate"

# EDLines and ELSED also provide timing when available
```

Expected results on 640×480 image (modern CPU):
- **LSD**: 25-50 fps
- **FLD**: 40-100 fps
- **Hough**: 50-125 fps
- **EDLines**: 60-200 fps
- **ELSED**: 100-300 fps

---

## Integration with ZenSLAM

To integrate any of these detectors into ZenSLAM:

1. **Choose your detector** based on requirements:
   - **Keep LSD**: Already working, good balance
   - **Add FLD**: Easy vcpkg integration, faster
   - **Add EDLines**: Best performance, manual integration
   - **Add ELSED**: Most modern, manual integration

2. **Update CMakeLists.txt**:
   ```cmake
   option(ZENSLAM_USE_FLD "Use FLD instead of LSD" OFF)
   option(ZENSLAM_USE_EDLINES "Use EDLines (requires ED_Lib)" OFF)
   option(ZENSLAM_USE_ELSED "Use ELSED (requires ELSED)" OFF)
   ```

3. **Modify grid_detector.cpp**:
   - Add conditional compilation based on detector choice
   - Convert detector output to `keyline` format
   - See example code in each file for conversion patterns

4. **Test thoroughly**:
   - Run unit tests
   - Verify matching still works
   - Check tracking performance
   - Validate SLAM output quality

---

## Troubleshooting

### OpenCV not found
```bash
# Install via vcpkg
vcpkg install opencv[contrib]

# Or via package manager
sudo apt install libopencv-dev libopencv-contrib-dev  # Ubuntu/Debian
```

### Missing line_descriptor module
```bash
# Ensure opencv is installed with contrib
vcpkg install opencv[contrib]  # Note: [contrib] is required
```

### ED_Lib compilation errors
```bash
# ED_Lib requires specific OpenCV version
# Check ED_Lib documentation for compatibility
```

### ELSED compilation errors
```bash
# ELSED requires C++11 or later
g++ -std=c++14 ...  # Use C++14 or C++17
```

---

## See Also

- [Line Segment Libraries Research](../line_segment_libraries_research.md) - Comprehensive comparison
- [Keyline Tracking](../keyline_tracking.md) - How lines are tracked in ZenSLAM
- [Feature Pipeline](../feature_pipeline.md) - Overall detection pipeline
- [Build System](../build_system.md) - CMake configuration

---

## Contributing

To add a new example:
1. Follow the existing code structure
2. Include detailed comments and timing information
3. Provide compilation instructions
4. Add entry to this README
5. Update the comparison table

---

## References

- **LSD**: Von Gioi et al., "LSD: A Fast Line Segment Detector" (2010)
- **FLD**: OpenCV ximgproc module documentation
- **EDLines**: Topal & Akinlar, "Edge Drawing" (2012)
- **ELSED**: Suárez et al., "ELSED: Enhanced Line SEgment Drawing" (2022)
- **Hough**: Classical computer vision algorithm

