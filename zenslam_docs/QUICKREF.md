# ZenSLAM Documentation Quick Reference

Fast lookup for common tasks and topics.

## I want to...

### Get Started
- **Build the project** → [Build System](getting-started/build-system.md)
- **Understand the architecture** → [Overview](getting-started/overview.md)
- **See what's planned** → [Roadmap](getting-started/roadmap.md)

### Work with Cameras
- **Load calibration files** → [Calibration](core/calibration.md)
- **Detect features** → [Feature Pipeline](core/feature-pipeline.md)
- **Match stereo pairs** → [Stereo Matching](core/stereo-matching.md)
- **Filter bad matches** → [Epipolar Filtering](advanced/epipolar-filtering.md)
- **Reconstruct 3D points** → [Triangulation](advanced/triangulation.md)

### Track Features
- **Temporal tracking** → [Tracking](core/tracking.md)
- **Track line features** → [Keylines](features/keylines.md)
- **Measure tracking performance** → [Tracking Statistics](features/tracking-statistics.md)

### Estimate Pose
- **Multi-method fusion** → [Pose Estimation](advanced/pose-estimation.md)
- **Understand uncertainty** → [Pose Estimation § Covariance](advanced/pose-estimation.md#covariance-estimation)

### Work with IMU
- **Integrate IMU measurements** → [IMU Preintegration](core/imu-preintegration.md)
- **Configure IMU parameters** → [IMU Calibration](advanced/imu-calibration.md)

### Configure Matchers
- **Matcher strategies** → [Matcher Options](advanced/matcher-options.md)
- **FLANN setup** → [FLANN Matcher Guide](development/flann-matcher-guide.md)

### Debug and Develop
- **Implementation patterns** → [Implementation Notes](development/implementation-notes.md)
- **Fusion debugging** → [Implementation Notes § Weighted Fusion](development/implementation-notes.md#weighted-fusion-debugging)
- **Covariance patterns** → [Implementation Notes § Pose Covariance](development/implementation-notes.md#pose-covariance-implementation)

### Research Libraries
- **Line detection libraries** → [Line Segment Libraries](research/line-segments.md)

## By Component

| Component | Core Docs | Advanced | Development |
|-----------|-----------|----------|-------------|
| **Detection** | [Feature Pipeline](core/feature-pipeline.md) | - | - |
| **Matching** | [Stereo Matching](core/stereo-matching.md) | [Matcher Options](advanced/matcher-options.md) | [FLANN Guide](development/flann-matcher-guide.md) |
| **Tracking** | [Tracking](core/tracking.md) | - | [Statistics](features/tracking-statistics.md) |
| **Lines** | - | - | [Keylines](features/keylines.md) |
| **Geometry** | - | [Epipolar Filtering](advanced/epipolar-filtering.md)<br>[Triangulation](advanced/triangulation.md) | - |
| **Pose** | - | [Pose Estimation](advanced/pose-estimation.md) | [Debug Notes](development/implementation-notes.md) |
| **IMU** | [Preintegration](core/imu-preintegration.md) | [Calibration](advanced/imu-calibration.md) | - |
| **Calibration** | [Calibration](core/calibration.md) | - | - |

## By Difficulty

### Beginner
Start here if new to the codebase:
1. [Overview](getting-started/overview.md) - Understand the pipeline
2. [Build System](getting-started/build-system.md) - Get it running
3. [Feature Pipeline](core/feature-pipeline.md) - First algorithm
4. [Calibration](core/calibration.md) - Data input

### Intermediate
Once comfortable with basics:
1. [Stereo Matching](core/stereo-matching.md) - Correspondence
2. [Tracking](core/tracking.md) - Temporal aspects
3. [Epipolar Filtering](advanced/epipolar-filtering.md) - Geometry
4. [Triangulation](advanced/triangulation.md) - 3D reconstruction

### Advanced
For deep dives:
1. [Pose Estimation](advanced/pose-estimation.md) - Multi-method fusion
2. [IMU Preintegration](core/imu-preintegration.md) - UGPM integration
3. [Keylines](features/keylines.md) - Line features
4. [Implementation Notes](development/implementation-notes.md) - Patterns

## Common Topics

### Configuration
- [Build System](getting-started/build-system.md) - CMake and vcpkg
- [Matcher Options](advanced/matcher-options.md) - Matching parameters
- [IMU Calibration](advanced/imu-calibration.md) - IMU parameters

### Algorithms
- [Feature Pipeline](core/feature-pipeline.md) - Grid detection
- [Stereo Matching](core/stereo-matching.md) - Descriptor matching
- [Pose Estimation](advanced/pose-estimation.md) - Weighted fusion
- [IMU Preintegration](core/imu-preintegration.md) - UGPM method

### Performance
- [Tracking Statistics](features/tracking-statistics.md) - Metrics and benchmarks
- [Implementation Notes](development/implementation-notes.md) - Optimization patterns

### Research
- [Line Segment Libraries](research/line-segments.md) - Library comparison
- [Roadmap](getting-started/roadmap.md) - Future directions

---

**Can't find what you need?** Check the [full documentation index](README.md).
