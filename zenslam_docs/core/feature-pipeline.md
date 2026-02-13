# Feature Pipeline

ZenSLAM emphasizes a uniform spatial distribution of keypoints to stabilize early geometry estimates.

## Grid Detector Wrapper

`grid_detector` subdivides the image into fixed-size cells (configured via options) and:

1. Runs the underlying detector in each cell.
2. Selects the highest-response keypoint per cell.

This reduces clustering in high-texture regions and ensures coverage.

## Detector Backends

- FAST (corner detection; fast & dense)
- ORB / SIFT descriptors (currently using SIFT for description in the sample pipeline)

The detector and descriptor steps are separated: detect -> compute descriptors.

## Keypoint Data

Stored in `mono_frame.keypoints` as `std::vector<cv::KeyPoint>`.

## Planned Enhancements

- Dynamic cell sizing (adaptive to feature density)
- Non-max suppression across cell borders
- Multi-scale pyramid support
- Learned detectors (later stage)
