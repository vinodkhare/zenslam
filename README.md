## ZenSLAM

ZenSLAM is an experimental stereo SLAM playground. It currently provides:

* A core C++23 library (`zenslam_core`) with calibration parsing, frame abstractions, feature detection (grid +
  FAST/ORB/SIFT backends), stereo matching, epipolar filtering, triangulation utilities, threading helpers, and option
  parsing.
* An application layer (`zenslam_app`) with a simple GUI (HelloImGui + OpenCV viz) for visualizing frames, matches,
  and (work-in-progress) reconstructed 3D points.
* A Python utility (`zenslam_py/scripts/bag_to_images.py`) to extract image sequences from ROS1/ROS2 bag files using
  `rosbags` (no ROS install needed).
* A Catch2 based test target (`zenslam_tests`) for unit / regression tests.

Status: early / evolving. Expect APIs to change.

---

## Coding Philosophy

* Start with the most obvious solution and then refactor
* The most obvious solution will often be versbose - let this be. Once you have enough code, patterns for refactoring
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

Example vcpkg install (adjust triplet, add --overlay-ports if needed):

```bash
./vcpkg install \
  boost-program-options \
  concurrentqueue \
  'hello-imgui[glfw-binding,opengl3-binding]' \
  magic-enum \
  'opencv[contrib,nonfree,vtk]' \
  spdlog \
  yaml-cpp \
  catch2
```

Ensure `CMAKE_TOOLCHAIN_FILE` points to your vcpkg toolchain when configuring.

---

## Configure & Build (All Targets)

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
      -DCMAKE_TOOLCHAIN_FILE="/path/to/vcpkg.cmake"
cmake --build build -j
```

Run the app:

```bash
./build/zenslam_app/zenslam_app   # or from install prefix after `cmake --install build`
```

Run tests:

```bash
ctest --test-dir build -j --output-on-failure
```

---

## Core Features (Implemented So Far)

* YAML stereo calibration parsing (Kalibr-style) with fundamental & projection matrix helpers.
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

Extended docs live in `zenslam_docs/`:

| Topic                                                    | Summary                                                    |
|----------------------------------------------------------|------------------------------------------------------------|
| [Overview](zenslam_docs/overview.md)                     | High-level architecture & data flow                        |
| [Calibration](zenslam_docs/calibration.md)               | Parsing Kalibr YAML, intrinsics, projection, F computation |
| [Feature Pipeline](zenslam_docs/feature_pipeline.md)     | Grid-based detection + descriptor backends                 |
| [Stereo Matching](zenslam_docs/stereo_matching.md)       | Descriptor matching strategy & future improvements         |
| [Epipolar Filtering](zenslam_docs/epipolar_filtering.md) | Geometric pruning via fundamental matrix distances         |
| [Triangulation](zenslam_docs/triangulation.md)           | 3D reconstruction from filtered correspondences            |
| [Tracking](zenslam_docs/tracking.md)                     | Temporal KLT plans & persistent tracks roadmap             |
| [Roadmap](zenslam_docs/roadmap.md)                       | Short / medium / long-term goals                           |
| [Build System](zenslam_docs/build_system.md)             | CMake, dependencies, dev environment                       |

These documents are evolving—contributions and corrections are welcome.

---

## Roadmap (Short-Term)

* Maintain persistent keypoint tracks (KLT + descriptor refresh).
* Basic pose estimation (PnP / essential decomposition).
* Map point management (prune, merge, re-triangulate).
* Visualization of 3D points in-app (color by track length / reprojection error).
* Benchmark scripts & dataset loaders (TUM-VI, EuRoC).

---

## Contributing

Early stage: feel free to open issues / PRs for ideas or small improvements. Larger architectural changes: start a
discussion first.

---

## License

See `LICENSE`.

---

## Acknowledgements

Built with fantastic open-source libraries: OpenCV, VTK, HelloImGui, yaml-cpp, spdlog, magic_enum, Catch2, and more.

