# Triangulation

We reconstruct 3D points from filtered stereo correspondences.

## Inputs
- Filtered matches (after epipolar pruning)
- Projection matrices `P0`, `P1`
- Normalized / undistorted pixel coordinates

## Method
`cv::triangulatePoints(P0, P1, pts_l, pts_r, X4)` returns homogeneous coordinates. We convert to Euclidean:
```
X = (X/W, Y/W, Z/W)
```

## Current Simplifications
- Baseline assumed from calibration (no runtime refinement).
- No cheirality check yet (future: ensure points lie in front of both cameras).
- No bundle adjustment / reprojection error filtering.

## Planned
- Add reprojection error thresholding
- Maintain per-point track length & uncertainty
- Introduce inverse-depth parameterization for downstream pose estimation
