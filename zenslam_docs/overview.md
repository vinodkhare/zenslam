# ZenSLAM Documentation Overview

This folder contains deeper, evolving documentation for the ZenSLAM project.

## Contents

- `overview.md` (this file): High-level architecture and data flow.
- `calibration.md`: Camera model & calibration parsing.
- `feature_pipeline.md`: Keypoint detection & description (grid + detector backend).
- `stereo_matching.md`: Descriptor matching & match representation.
- `epipolar_filtering.md`: Fundamental-matrix based geometric consistency filtering.
- `triangulation.md`: 3D point reconstruction from filtered stereo pairs.
- `tracking.md`: Temporal tracking (current + planned KLT + descriptor refresh).
- `roadmap.md`: Near-term and longer-term algorithmic goals.
- `build_system.md`: Build, dependencies, and dev environment tips.

> Status: Early-stage; expect rapid iteration and restructuring.

## High-Level Pipeline (Current)

```
Dataset Folder -> Stereo Frame Loader -> Undistort -> Grid Feature Detection
  -> Descriptor Extraction -> Brute Force Matching -> Epipolar Filtering
  -> Triangulation -> (WIP) Temporal Tracking -> Visualization / Output
```

## Core Principles

1. Keep components decoupled & testable.
2. Start simple; verify each stage before adding complexity.
3. Prefer explicit data structures over opaque side effects.
4. Log richly (with spdlog) for introspection; later gate verbosity.

## Data Structures (Simplified)

| Structure       | Purpose                                |
|-----------------|----------------------------------------|
| `mono_frame`    | Single camera image + keypoints        |
| `frame::stereo`  | Paired left/right frames + matches, 3D |
| `calibration`   | Intrinsics, distortion, pose           |
| `grid_detector` | Wraps detector ensuring spatial spread |
| `slam_thread`   | Drives the ingest + processing loop    |

## Extension Points

- Swap detector backend (FAST/ORB/SIFT/…)
- Use different match strategies (cross-check, ratio, GMS later)
- Add robust pose estimation (PnP / essential decomposition)
- Integrate map & keyframe management

See individual pages for details.
