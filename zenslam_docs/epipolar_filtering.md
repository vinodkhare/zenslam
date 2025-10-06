# Epipolar Filtering

Raw descriptor matches often include geometrically inconsistent pairs. Given known stereo calibration, we enforce the epipolar constraint.

## Fundamental Matrix
Computed once from the two `calibration` objects:
```
F = K2^{-T} [t]_x R K1^{-1}
```

## Filtering Criterion
For a match `(x_l, x_r)`:
1. Compute epipolar lines `l_r = F * x_l` and `l_l = F^T * x_r`.
2. Measure perpendicular distances:
```
d_l = |l_l^T x_l| / sqrt(a_l^2 + b_l^2)
d_r = |l_r^T x_r| / sqrt(a_r^2 + b_r^2)
```
3. Accept if both below threshold (configurable `epipolar_threshold`).

Filtered results stored in `stereo_frame.filtered`.

## Notes
- Current method is deterministic; does not (yet) leverage robust statistical thresholds.
- Future: incorporate Sampson distance or use OpenCV `findFundamentalMat` with RANSAC for additional resiliency even if calibration is known (to handle slight sync/calib drift).
