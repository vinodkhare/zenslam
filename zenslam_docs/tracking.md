# Temporal Tracking

Temporal feature tracking links observations across frames to enable motion & structure estimation.

## Current State
- Initial prototype (commented / WIP) uses KLT (`cv::calcOpticalFlowPyrLK`) to propagate 2D positions frame-to-frame before re-detection.
- Not yet integrated with a persistent ID or track management structure.

## Planned Pipeline
1. Detect features on keyframes.
2. Track with KLT on intermediate frames.
3. Refresh descriptors when track drifts or at keyframe boundaries.
4. Cull lost / low-quality tracks (large error, short lifespan).
5. Feed surviving multi-view correspondences into pose estimator + triangulation refinement.

## Enhancements Considered
- Forward-backward KLT consistency check
- Robust track scoring (age, gradient magnitude, reprojection stability)
- Re-detect in depleted regions only (mask existing tracks)
