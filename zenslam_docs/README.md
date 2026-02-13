# ZenSLAM Documentation

Comprehensive documentation for the ZenSLAM visual-inertial SLAM system.

> **Quick Navigation:** See [QUICKREF.md](QUICKREF.md) for fast topic lookup.

## 📚 Documentation Structure

### [Getting Started](getting-started/)
Entry point for new users and developers.

- **[Overview](getting-started/overview.md)** - System architecture and pipeline
- **[Build System](getting-started/build-system.md)** - CMake, dependencies, and development setup
- **[Roadmap](getting-started/roadmap.md)** - Development priorities and future plans

### [Core Concepts](core/)
Fundamental SLAM algorithms and components.

- **[Calibration](core/calibration.md)** - Camera models and calibration parsing
- **[Feature Pipeline](core/feature-pipeline.md)** - Keypoint detection and grid-based distribution
- **[Stereo Matching](core/stereo-matching.md)** - Descriptor matching and correspondence
- **[Tracking](core/tracking.md)** - Temporal feature tracking strategies
- **[IMU Preintegration](core/imu-preintegration.md)** - UGPM-based IMU integration

### [Advanced Topics](advanced/)
Detailed technical implementations.

- **[Epipolar Filtering](advanced/epipolar-filtering.md)** - Geometric consistency filtering
- **[Triangulation](advanced/triangulation.md)** - 3D point reconstruction
- **[Pose Estimation](advanced/pose-estimation.md)** - Weighted fusion and pose recovery
- **[Matcher Options](advanced/matcher-options.md)** - Matching strategies and parameters
- **[IMU Calibration](advanced/imu-calibration.md)** - IMU intrinsic calibration

### [Features](features/)
Specialized feature implementations.

- **[Keylines](features/keylines.md)** - Line feature detection, tracking, and masking
- **[Tracking Statistics](features/tracking-statistics.md)** - Performance metrics and analysis

### [Development](development/)
Implementation notes and debugging guides.

- **[Implementation Notes](development/implementation-notes.md)** - Covariance and fusion debugging
- **[FLANN Matcher Guide](development/flann-matcher-guide.md)** - FLANN integration details

### [Research](research/)
Experimental work and library evaluations.

- **[Line Segment Libraries](research/line-segments.md)** - Comparison of line detection libraries

## 🚀 Quick Start

1. Read the [Overview](getting-started/overview.md) for system architecture
2. Follow [Build System](getting-started/build-system.md) to set up your environment
3. Explore [Core Concepts](core/) to understand the SLAM pipeline
4. Check [Roadmap](getting-started/roadmap.md) for current development status

## 📝 Contributing

When adding documentation:
- Place user-facing docs in `getting-started/` or `core/`
- Technical deep-dives belong in `advanced/`
- Debug notes and session logs go in `development/`
- Experimental comparisons go in `research/`

## 📋 Status

Documentation is actively evolving. Expect restructuring as the system matures.
