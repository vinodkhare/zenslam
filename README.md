# ZenSLAM

## What is ZenSLAM?

ZenSLAM is a "nothing special" SLAM system. In the last couple of decades or so, SLAM has matured as a technology and has become inreasingly commoditized. Today it is possible to write a SLAM system using off-the-shelf components and libraries. ZenSLAM is an exploration of this idea: can we build a simple stereo SLAM system using well-known techniques and readily available libraries? ZenSLAM aims not for innovation, but for clarity, simplicity, and ease of understanding and use.

Status: early / evolving. Expect APIs to change.

![screenshot](zenslam_docs/images/screenshot.png)

---
## Coding Philosophy

* Start with the most obvious solution and then refactor
* The most obvious solution will often be verbose - let this be. Once you have enough code, patterns for refactoring
  will suggest themselves.
* Premature abstraction (like premature optimization) is the root of all Evil.
* Aim for simplicity: make things as simple as possible, but no simpler.
* Do not overdesign. Sometimes the best solution is a simple function or a plain structure.

### Refactoring

Refactoring is the process of taking existing code and reshaping it into a different form without changing what it does.
Why do we refactor when it doesn't change the behavior of the code? Refactoring is discovering a new way to look at your
code and what it does. Refactoring makes things easier to understand, to maintain and sometimes more efficient. But more
than that, refactoring helps you think about your problem is a new, different, more fertile way. One reason to refactor
is to open up new possibilities.

#### Lumping and Splitting

Lumping and splitting are two fundamental ways to refactor something. At this point in the project, several
opportunities for lumping and splitting present themselves.

The `utils` file has become too big—it could be split into multiple files. This would allow for faster compilation times
and easier maintenance. This is an example of splitting. But note how there is a judgment to be made here. How big is
too big? There is no principled answer to this question. One person may think that a 1000-line file is too big. Another
may think that 1000 lines is just fine. But despite this, we can kinda sorta agree about what too big is. A "perfect"
choice doesn't matter. *Any* choice will do.

Many such decisions need to be made during software development. These decisions are not strictly systematic. Of course,
you *could* systematize them. You could make a rule—no files larger than 1000 lines! But this seems arbitrary and
absurd. What if the file is 1001 lines? Is that extra one line really so bad? As a developer, I'm tempted to delete some
whitespace and check that file in. And ultimately, even if the file is 1001 lines, the time spent on this nitpicking is
not worth it.

It may seem like I'm going on pointlessly but you'd be suprised how much time is wasted in teams on such pointless rules
and decisions.

Detection time:
Non-Parallel: 0.032 s to 0.037 s
Parallel: 0.047 s to 0.047 s

Parallel is not an improvement!

---

## Repository Layout

```
zenslam_core/     # Core C++ library sources and headers (generic SLAM components)
zenslam_app/      # GUI / application layer (ImGui + Viz3d integration)
zenslam_tests/    # Catch2 tests
zenslam_options/  # Example/options data (YAML calibration & configs)
zenslam_py/       # Python helper scripts (bag extraction)
```

---

## Dependencies

Installed via vcpkg (recommended). Core components require:

* OpenCV (modules: core, imgproc, features2d, calib3d, video, viz, contrib/nonfree if using SIFT)
* Boost (program_options)
* yaml-cpp
* spdlog
* magic_enum
* Microsoft GSL
* concurrentqueue
* hello-imgui (GUI)
* VTK (pulled indirectly by OpenCV viz)
* Catch2 (tests)
* nanoflann
* glog
* Ceres Solver (with EigenSparse, LAPACK, Schur, SuiteSparse options)

Multi-line for ZSH:

```zsh
./vcpkg install boost-program-options
./vcpkg install catch2
./vcpkg install 'ceres[eigensparse,lapack,schur,suitesparse]'
./vcpkg install concurrentqueue
./vcpkg install gtsam
./vcpkg install 'hello-imgui[metal-binding,glfw-binding]'
./vcpkg install implot
./vcpkg install magic-enum
./vcpkg install ms-gsl
./vcpkg install nanoflann
./vcpkg install 'opencv[vtk]'
./vcpkg install rapidcsv
./vcpkg install spdlog
./vcpkg install yaml-cpp
```

Or, single-line for ZSH:

```zsh
./vcpkg install boost-program-options catch2 'ceres[eigensparse,lapack,schur,suitesparse]' concurrentqueue gtsam 'hello-imgui[metal-binding,glfw-binding]' implot magic-enum ms-gsl nanoflann 'opencv[vtk]' rapidcsv spdlog yaml-cpp
```

Ensure `CMAKE_TOOLCHAIN_FILE` points to your vcpkg toolchain when configuring.

---

## Configure & Build (All Targets)

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_TOOLCHAIN_FILE="/path/to/vcpkg.cmake"
cmake --build build -j
```

Build times:

| Date       | Debug        | Release      |
|------------|--------------|--------------|
| 2025-10-23 | 00:01:55.546 | 00:00:24.286 |

Run the app:

```bash
./build/zenslam_app/zenslam_app   # or from install prefix after `cmake --install build`
```

### Troubleshooting

After a MacOS update, you might start getting cmake or linker errors related to missing SDK files. To fix this, I created a symlink from the expected SDK path to the actual SDK path. E.g. `ln -s MacOSX26.1.sdk MacOSX26.0.sdk`.

### zenslam_app Options

All CLI options for `zenslam_app` are listed below. Boolean flags accept `true|false` (or just `--flag` to enable).

| Option                                   | Default           | Permissible values                                           | Description                                               |
| ---------------------------------------- | ----------------- | ------------------------------------------------------------ | --------------------------------------------------------- |
| `--options-file`                         | `options.yaml`    | Path                                                         | Options YAML file to load (CLI overrides YAML).           |
| `--log-level`                            | `trace`           | `trace`, `debug`, `info`, `warn`, `error`, `critical`, `off` | Logging verbosity.                                        |
| `--help`, `-h`                           | `false`           | Flag                                                         | Show help text and exit.                                  |
| `--version`, `-v`                        | `false`           | Flag                                                         | Print version and exit.                                   |
| `--folder-root`                          | `.`               | Path                                                         | Root folder for dataset.                                  |
| `--folder-left`                          | `cam0`            | Path                                                         | Left image folder (relative to root or absolute).         |
| `--folder-right`                         | `cam1`            | Path                                                         | Right image folder (relative to root or absolute).        |
| `--folder-timescale`                     | `1.0`             | $> 0$                                                        | Timescale applied to folder timestamps.                   |
| `--calibration-file`                     | `camchain.yaml`   | Path                                                         | Stereo calibration file path.                             |
| `--groundtruth-file`                     | `groundtruth.csv` | Path                                                         | Groundtruth file path.                                    |
| `--imu-calibration-file`                 | `imu_config.yaml` | Path                                                         | IMU calibration file path.                                |
| `--imu-file`                             | (empty)           | Path                                                         | IMU data CSV file path.                                   |
| `--folder-output`                        | `output`          | Path                                                         | Output folder for results.                                |
| `--clahe-enabled`                        | `false`           | `true`                                                       | `false`                                                   | Enable CLAHE preprocessing.      |
| `--use-parallel-detector`                | `true`            | `true`                                                       | `false`                                                   | Use parallel grid detector.      |
| `--stereo-rectify`                       | `false`           | `true`                                                       | `false`                                                   | Enable stereo rectification.     |
| `--epipolar-threshold`                   | `1.0`             | $\ge 0$                                                      | Epipolar distance threshold.                              |
| `--feature`                              | `FAST`            | `FAST`, `ORB`, `SIFT`                                        | Feature detector type.                                    |
| `--descriptor`                           | `ORB`             | `ORB`, `SIFT`, `FREAK`                                       | Descriptor type.                                          |
| `--matcher`                              | `BRUTE`           | `BRUTE`, `KNN`, `FLANN`                                      | Matcher backend.                                          |
| `--integrator-method`                    | `ugpm`            | `ugpm`, `lpm`                                                | IMU preintegration method.                                |
| `--matcher-ratio`                        | `0.8`             | $0 < x < 1$                                                  | Ratio test threshold for kNN matching.                    |
| `--fast-threshold`                       | `10`              | $\ge 0$                                                      | FAST detector threshold.                                  |
| `--threshold-3d3d`                       | `0.005`           | $\ge 0$                                                      | 3D-3D RANSAC threshold (meters).                          |
| `--threshold-3d2d`                       | `1.0`             | $\ge 0$                                                      | 3D-2D RANSAC threshold (pixels).                          |
| `--show-keypoints`                       | `true`            | `true`                                                       | `false`                                                   | Show keypoints in visualization. |
| `--show-keylines`                        | `true`            | `true`                                                       | `false`                                                   | Show keylines in visualization.  |
| `--keyline-single-color`                 | `0 255 0`         | `B G R` ints (0-255)                                         | Keyline single color (BGR).                               |
| `--keyline-match-color`                  | `0 0 255`         | `B G R` ints (0-255)                                         | Keyline match color (BGR).                                |
| `--keyline-thickness`                    | `1`               | $\ge 1$                                                      | Keyline line thickness (pixels).                          |
| `--keyline-min-disparity-px`             | `2.0`             | $\ge 0$                                                      | Min average disparity across keyline endpoints (pixels).  |
| `--keyline-min-triangulation-angle-deg`  | `15`              | $\ge 0$                                                      | Min triangulation angle at endpoints (degrees).           |
| `--triangulation_reprojection_threshold` | `1.0`             | $> 0$                                                        | Max average reprojection error across endpoints (pixels). |

### Running Benchmarks

The test suite includes Catch2 benchmarks comparing `cv::Mat` (CPU) vs `cv::UMat` (OpenCL) performance for feature detection and matching.

**Run all benchmarks:**
```bash
./build/zenslam_tests/hello_test "[benchmark]"
```

**Run specific benchmark:**
```bash
# Keypoint detection benchmark only
./build/zenslam_tests/hello_test "[benchmark]" -c "cv::Mat vs cv::UMat Keypoint Detection"

# Feature matching pipeline benchmark
./build/zenslam_tests/hello_test "[benchmark]" -c "cv::Mat vs cv::UMat Feature Matching"

# Grid detector sequential vs parallel benchmark
./build/zenslam_tests/hello_test "[benchmark]" -c "grid_detector::detect_keypoints vs detect_keypoints_par"
```

**Benchmark output options:**
```bash
# Detailed statistics with confidence intervals
./build/zenslam_tests/hello_test "[benchmark]" --benchmark-samples 100

# JSON output for analysis
./build/zenslam_tests/hello_test "[benchmark]" --reporter JSON::out=benchmark_results.json

# Minimal output
./build/zenslam_tests/hello_test "[benchmark]" --benchmark-no-analysis
```

### Tracking Statistics

See [tracking statistics](zenslam_docs/features/tracking-statistics.md) for detailed tracking performance metrics and comparisons across different configurations.

**Notes:**
- **UMat benchmarks** require OpenCL support (check with `cv::ocl::haveOpenCL()`)
  - First UMat run may be slower due to OpenCL initialization and kernel compilation
  - Performance varies by GPU/driver; integrated GPUs may not show speedup for small images
  - Use larger images or more iterations to see GPU benefits (adjust benchmark parameters in code)
- **Grid detector benchmarks** compare sequential vs parallel grid-based feature detection
  - `detect_keypoints`: Processes grid cells sequentially (original implementation)
  - `detect_keypoints_par`: Processes grid cells in parallel using `std::async`
  - Tests include empty grid (all new detections) and 50% occupancy scenarios
  - Expected speedup on multi-core systems: ~2-4× depending on grid size and CPU cores
  - Larger grid sizes (smaller cells) benefit more from parallelization

**Interpreting results:**
- **Mean time**: Average execution time per iteration
- **Low/High mean**: 95% confidence interval for the mean
- **Std dev**: Standard deviation of measurements
- Lower times indicate better performance; compare CPU vs GPU mean times to see speedup

---

## Parallel detector toggle

When enabled, keypoints are detected per-grid-cell using a parallel implementation that processes cells concurrently. On multi-core CPUs this often improves throughput; for very small images or light workloads, the sequential version can avoid threading overhead.

---

## Core Features (Implemented So Far)

* YAML stereo calibration parsing (Kalibr-style) with fundamental & projection matrix helpers.
* IMU calibration parsing with support for noise and bias parameters (accelerometer/gyroscope).
* Feature detection via grid-based wrapper (uniform spatial distribution) over FAST/SIFT/ORB.
* Descriptor extraction + brute force matching.
* Epipolar filtering using a fundamental matrix + distance threshold.
* Triangulation of filtered stereo correspondences into 3D points.
* (WIP) KLT optical flow tracking between consecutive frames.
* Utility helpers (string joins, undistortion, keypoint conversions, logging formatters for Affine3d).

---

## Python: Bag Image Extraction

Create venv & install:

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r zenslam_py/scripts/requirements.txt
```

Usage:

```bash
python zenslam_py/scripts/bag_to_images.py <bag_path> <output_dir> \
  [--topics /cam0/image_raw /cam1/image_raw] \
  [--image-format png|jpg] [--limit-per-topic N] [--flat] [--quiet]
```

Notes:

* Supports ROS1 `.bag` and ROS2 directory bags.
* Extracts `sensor_msgs/Image` + `CompressedImage`.
* Filenames encode (relative) timestamp.

---

## Development Tips

* Enable compile commands for clangd / IDE: `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON`.
* Suppress verbose VTK viz logs (optional):
  ```cpp
  #include <vtkLogger.h>
  vtkLogger::SetStderrVerbosity(vtkLogger::VERBOSITY_ERROR);
  ```
* Projection matrix built via `K * [R|t]` using minor extraction.
* Fundamental matrix: `F = K2^{-T} [t]_x R K1^{-1}` derived from relative pose.

---

## Documentation

Comprehensive documentation is available in [`zenslam_docs/`](zenslam_docs/) with organized categories:

### Getting Started
- **[Overview](zenslam_docs/getting-started/overview.md)** - System architecture and pipeline
- **[Build System](zenslam_docs/getting-started/build-system.md)** - CMake, dependencies, development environment
- **[Roadmap](zenslam_docs/getting-started/roadmap.md)** - Development priorities and future plans

### Core Concepts
- **[Calibration](zenslam_docs/core/calibration.md)** - Camera models and Kalibr YAML parsing
- **[Feature Pipeline](zenslam_docs/core/feature-pipeline.md)** - Grid-based keypoint detection
- **[Stereo Matching](zenslam_docs/core/stereo-matching.md)** - Descriptor matching strategies
- **[Tracking](zenslam_docs/core/tracking.md)** - Temporal correspondence and KLT tracking
- **[IMU Preintegration](zenslam_docs/core/imu-preintegration.md)** - UGPM-based IMU integration

### Advanced Topics
- **[Epipolar Filtering](zenslam_docs/advanced/epipolar-filtering.md)** - Geometric consistency pruning
- **[Triangulation](zenslam_docs/advanced/triangulation.md)** - 3D point reconstruction
- **[Pose Estimation](zenslam_docs/advanced/pose-estimation.md)** - Multi-method weighted fusion
- **[IMU Calibration](zenslam_docs/advanced/imu-calibration.md)** - IMU intrinsic parameters
- **[Matcher Options](zenslam_docs/advanced/matcher-options.md)** - Matching configurations

### Features & Research
- **[Keylines](zenslam_docs/features/keylines.md)** - Line feature detection and tracking
- **[Tracking Statistics](zenslam_docs/features/tracking-statistics.md)** - Performance metrics
- **[Line Segment Libraries](zenslam_docs/research/line-segments.md)** - Library comparisons

See the [full documentation index](zenslam_docs/README.md) for complete navigation.

These documents are evolving—contributions and corrections are welcome.

---

## Contributing

Early stage: feel free to open issues / PRs for ideas or small improvements. Larger architectural changes: start a
discussion first.

---

## License

See [`LICENSE`](LICENSE).