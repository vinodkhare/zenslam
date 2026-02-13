# Pose Estimation with Weighted Fusion

Multi-method pose estimation with intelligent weighted fusion for improved robustness and accuracy.

## Overview

ZenSLAM estimates camera pose using multiple geometric methods and fuses them via weighted averaging based on confidence metrics. This approach:

- **Improves robustness** in challenging scenarios
- **Quantifies uncertainty** with confidence scores
- **Leverages all available data** instead of single best method
- **Provides intelligent fallback** for low-feature environments

## Architecture

### Pose Estimation Methods

Five methods provide pose estimates:

| Method | Input | Solver | Use Case |
|--------|-------|--------|----------|
| **3D-3D Points** | Triangulated points | Umeyama/ICP | Sufficient 3D points |
| **3D-2D Points** | Map points + 2D detections | PnP RANSAC | Map available |
| **2D-2D Points** | Stereo correspondences | Essential matrix | No prior map |
| **3D-3D Lines** | Triangulated line endpoints | Umeyama | Linear features |
| **3D-2D Lines** | Map lines + 2D detections | PnP variants | Line-based tracking |

### Data Structure

```cpp
struct weighted_pose_result {
    cv::Affine3d pose;                    // Fused pose estimate
    double       confidence { 0.0 };      // Overall confidence (0-1)
    int          total_inliers { 0 };     // Total inliers across methods
    
    // Covariance information
    cv::Matx66d  pose_covariance { };     // 6x6 SE(3) uncertainty
    double       translation_std { 0.0 }; // Translation std dev (meters)
    double       rotation_std { 0.0 };    // Rotation std dev (radians)
    bool         has_valid_covariance { false };
    
    // Individual method contributions
    double       weight_3d3d { 0.0 };
    double       weight_3d2d { 0.0 };
    double       weight_2d2d { 0.0 };
    double       weight_3d3d_lines { 0.0 };
    double       weight_3d2d_lines { 0.0 };
    
    // Best contributing method
    std::string  best_method;
    size_t       best_method_inliers { 0 };
};
```

## Weight Calculation

### Confidence Formula

For each method:

```
confidence = (inlier_ratio * 0.4 + error_quality * 0.4 + inlier_boost * 0.2) * type_weight
```

**Components:**

1. **Inlier Ratio** (40% weight)
   - `inliers / total_correspondences`
   - Returns 0 if < 3 inliers (insufficient data)
   - Heavy penalty if < 30% (low-quality matches)

2. **Error Quality** (40% weight)
   - Outlier detection: rejects errors > mean + 2σ
   - Exponential decay: `exp(-mean_error / error_scale)`
   - **Error scales:**
     - 3D methods: 0.1 meters
     - 2D methods: 3.0 pixels

3. **Inlier Boost** (20% weight)
   - `min(1.0, inlier_count / 50)`
   - Prefers methods with more absolute inliers
   - Normalized to 50 as reference count

4. **Type Weight**
   - Point-based methods: 1.0
   - Line-based methods: 0.9 (slightly less reliable)

### Normalization

Weights are normalized to sum to 1.0:

```cpp
double total_weight = w_3d3d + w_3d2d + w_2d2d + w_lines_3d3d + w_lines_3d2d;
if (total_weight > 0) {
    each_weight /= total_weight;
}
```

## Pose Fusion

### Translation Fusion

Weighted average across all methods:

```cpp
fused_translation = w1 * t1 + w2 * t2 + ... + wn * tn
```

### Rotation Fusion

Selected from the highest-confidence method (not averaged):

```cpp
fused_rotation = rotation_from_best_method
```

**Rationale:** Rotation averaging is mathematically complex and error-prone. Selecting from best method:
- Preserves rotation matrix validity
- Avoids gimbal lock
- Prevents interpolation artifacts

## Covariance Estimation

### Per-Method Uncertainty

```cpp
static auto compute_pose_covariance(
    const pose_data& pose, 
    double method_weight, 
    const std::string& method_type
) -> std::pair<double, double>
```

Returns `{translation_std, rotation_std}`:

- **3D methods:** Use reprojection/fitting errors directly
- **2D methods:** Scale pixel errors (1 pixel ≈ 0.01m at 1m depth)
- **Clamping:** Reasonable ranges to avoid outlier uncertainties

### Fused Uncertainty

Weighted averaging of individual uncertainties:

```cpp
fused.translation_std = w1 * std1_t + w2 * std2_t + ...
fused.rotation_std = w1 * std1_r + w2 * std2_r + ...
```

### Covariance Matrix

Diagonal 6×6 matrix in SE(3):

```cpp
// [rotation (3×3) | 0]
// [0 | translation (3×3)]
cv::Matx66d covariance = cv::Matx66d::zeros();
covariance(0,0) = covariance(1,1) = covariance(2,2) = rotation_std²;
covariance(3,3) = covariance(4,4) = covariance(5,5) = translation_std²;
```

## Usage in SLAM Thread

```cpp
// Estimate pose with all methods
auto estimate_result = estimator.estimate_pose(system[0], tracked);

// Apply weighted fusion
auto weighted_result = estimator.estimate_pose_weighted(estimate_result);

// Fallback strategy
cv::Affine3d chosen_pose;
if (weighted_result.confidence > 0.15 && weighted_result.total_inliers >= 6) {
    // Use fused pose
    chosen_pose = weighted_result.pose;
} else if (estimate_result.best_inliers >= 5) {
    // Use best single method
    chosen_pose = estimate_result.chosen_pose;
} else {
    // Last resort: motion prediction
    chosen_pose = pose_predicted;
}
```

## Debugging and Tuning

### Enable Detailed Logging

```cpp
SPDLOG_SET_LEVEL(spdlog::level::trace);
```

**Log output includes:**
- Per-method inlier counts and ratios
- Individual confidence weights
- Best contributing method
- Overall fusion confidence
- Weight distribution

### Diagnostic Indicators

**✅ Good signs:**
- Best method has highest weight
- Total inliers > 50
- Confidence between 0.5-0.95
- Weights sum to 1.0

**❌ Red flags:**
- Low confidence (< 0.2) despite good inliers → formula issue
- One method > 99% weight → not fusing
- All methods equal weight → all poor quality
- Confidence/best method mismatch → logic error

### Tuning Parameters

**Inlier ratio threshold:**
```cpp
if (inlier_ratio < 0.3) return 0.01 * inlier_ratio;  // Adjust 0.3
```

**Error scale (affects error_quality):**
```cpp
const double error_scale = 3.0;  // Increase for tolerance
```

**Component weights:**
```cpp
// Try 0.5/0.3/0.2 if inlier ratio should dominate
confidence = inlier_ratio * 0.4 + error_quality * 0.4 + inlier_boost * 0.2;
```

**Confidence threshold:**
```cpp
if (weighted_result.confidence < 0.15) // Adjust threshold
```

**Type weighting:**
```cpp
if (method_type.find("lines") != std::string::npos)
    type_weight = 0.9;  // Reduce if lines problematic
```

## Known Issues and Fixes

### Issue: Fused Pose Not Used ✅ FIXED
**Problem:** Weighted pose computed but ignored.  
**Fix:** Use `weighted_result.pose` as primary estimate.

### Issue: Rotation Averaging ✅ FIXED
**Problem:** Direct averaging of rotation vectors invalid.  
**Fix:** Select rotation from highest-confidence method.

### Issue: Weak Weight Calculation ✅ FIXED
**Problem:** Methods with few inliers got undeserved credit.  
**Fix:** Minimum 3 inliers, penalties for low ratios, better component weighting.

### Issue: Poor Fallback Logic ✅ FIXED
**Problem:** Premature fallback to motion prediction.  
**Fix:** Tiered fallback (fused → best single → prediction).

## Future Enhancements

- **Quaternion SLERP** for rotation interpolation
- **Adaptive weighting** based on scene characteristics
- **Covariance propagation** through Kalman filtering
- **Outlier-aware fusion** with robust statistics
- **Bundle adjustment** integration
