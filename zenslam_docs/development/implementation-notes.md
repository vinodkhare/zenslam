# Implementation Notes and Debug Sessions

Development notes, debugging sessions, and implementation patterns discovered during ZenSLAM development.

## Pose Covariance Implementation

### Overview

Implemented pose covariance estimation for weighted fusion to quantify uncertainty and enable intelligent decision-making.

### Architecture Patterns

#### 1. Helper Functions for Uncertainty

**Location:** `zenslam_core/source/estimator.cpp`

```cpp
static auto compute_pose_covariance(
    const pose_data& pose, 
    double method_weight, 
    const std::string& method_type
) -> std::pair<double, double>
```

**Key insights:**
- Static function keeps calculation self-contained and testable
- Returns `{translation_std, rotation_std}` for structured binding
- Handles method-specific error scaling (3D errors in meters, 2D in pixels)
- Pixel-to-meter conversion: ~0.01m/pixel at 1m depth
- Clamps output to reasonable ranges

#### 2. Result Struct for Multi-Value Returns

**Location:** `zenslam_core/include/zenslam/estimator.h`

```cpp
struct weighted_pose_result {
    cv::Matx66d  pose_covariance { };        // 6×6 SE(3) uncertainty
    double       translation_std { 0.0 };    // Translation std dev (meters)
    double       rotation_std { 0.0 };       // Rotation std dev (radians)
    bool         has_valid_covariance { false };
};
```

**Lesson:** Group uncertainty fields near confidence/weight fields for logical cohesion.

#### 3. Weighted Averaging of Uncertainties 

**Location:** `zenslam_core/source/estimator.cpp`

```cpp
fused.translation_std = w1 * std1_t + w2 * std2_t + ...
fused.rotation_std = w1 * std1_r + w2 * std2_r + ...
```

**Key insights:**
- Weights from method confidence sum to ~1.0
- High-confidence methods dominate (more reliable)
- Maintains coupling between confidence and uncertainty

#### 4. Diagonal Covariance Matrix

**Location:** `zenslam_core/source/estimator.cpp`

```cpp
fused.pose_covariance = cv::Matx66d::zeros();
const double trans_var = translation_std²;
const double rot_var = rotation_std²;

// Diagonal elements only (assumes independence)
fused.pose_covariance(0,0) = trans_var;  // x
fused.pose_covariance(1,1) = trans_var;  // y
fused.pose_covariance(2,2) = trans_var;  // z
fused.pose_covariance(3,3) = rot_var;    // roll
fused.pose_covariance(4,4) = rot_var;    // pitch
fused.pose_covariance(5,5) = rot_var;    // yaw
```

**Lesson:** Diagonal covariance is simplified but valid for initial implementation. Upgrade to full covariance later if cross-terms become important.

### Integration Strategy

#### Smart Fallback Using Covariance

**Three-tier decision logic:**

1. **Primary:** Use weighted fusion if:
   - `has_valid_covariance == true` (≥5 inliers)
   - `translation_std ≤ 0.5m` (uncertainty threshold)
   - `confidence ≥ 0.15` (fusion quality)
   - `total_inliers ≥ 6` (minimum data)

2. **Secondary:** Use best single method if primary fails
   - Requires ≥5 inliers

3. **Fallback:** Motion prediction if all else fails

**Key insight:** Translation uncertainty (0.5m threshold) is more concrete and actionable than abstract confidence scores.

#### Logging Strategy

```cpp
SPDLOG_INFO("Pose uncertainty estimates:");
if (weighted_result.has_valid_covariance) {
    SPDLOG_INFO("  Translation std: {:.4f}m, Rotation std: {:.6f}rad ({:.4f}deg)",
        weighted_result.translation_std, 
        weighted_result.rotation_std,
        weighted_result.rotation_std * 180.0 / M_PI);
}
```

**Lesson:** Include both metric and human-readable units for debugging.

### Mathematical Notes

#### Error-to-Uncertainty Mapping

```
base_uncertainty = (1 - inlier_ratio) / sqrt(inlier_ratio)
error_uncertainty = std_dev(errors)
combined = sqrt(error_uncertainty² + ratio_uncertainty²)
```

Combines information from inlier count and error magnitude.

#### Method-Specific Scaling

- **3D-3D:** Errors in meters, use directly
- **3D-2D:** Pixel errors scaled by ~0.01m/pixel
- **2D-2D:** Pixel errors scaled similarly

**Future:** Calibrate pixel-to-meter from dataset calibration.

#### Rotation Uncertainty

```
rotation_std = 0.1 * translation_std
```

Rough approximation: 1cm translation error ≈ 1 milliradian rotation error.

#### Why Not Vector Averaging for Rotation?

**Wrong approach:**
```cpp
// Rotation vectors don't form vector space
rotation_vec = w1 * vec1 + w2 * vec2  // Risk of gimbal lock
```

**Correct approach:**
```cpp
// Select rotation from highest-weight method
rotation = weighted_results[best_method].rotation
```

**Lesson:** SE(3) doesn't support linear rotation interpolation. Alternatives: (1) Select best, (2) Quaternion SLERP, (3) Riemannian manifold methods.

## Weighted Fusion Debugging

### Problems and Fixes

#### Problem 1: Fused Pose Not Used ✅
**Issue:** Weighted pose computed but ignored; code used single best method.  
**Fix:** Changed `slam_thread.cpp` to use `weighted_result.pose`.

#### Problem 2: Rotation Averaging ✅
**Issue:** Direct averaging of rotation vectors causes invalid matrices, gimbal lock.  
**Fix:** Select rotation from highest-confidence method.

#### Problem 3: Weak Weight Calculation ✅
**Original issues:**
- Methods with <3 inliers got credit
- 30% inlier ratio got significant weight
- Error quality defaulted to 1.0 incorrectly
- 0.6/0.3 split over-distributed weight

**Fix:** 
- Minimum 3 inliers requirement
- Heavy penalty for ratios <30%
- Outlier detection in errors
- Better weighting: 0.4/0.4/0.2

#### Problem 4: Poor Fallback Logic ✅
**Issue:** Premature fallback at <10 inliers discarded good estimates.  
**Fix:** Tiered fallback (fusion → best single → prediction).

### Debugging Techniques

#### Enable Trace Logging

```cpp
SPDLOG_SET_LEVEL(spdlog::level::trace);
```

**Output includes:**
- Per-method inlier counts and ratios
- Individual confidence weights
- Best contributing method
- Overall fusion confidence
- Weight breakdown

#### Comparative Analysis

```cpp
// Log pose difference
cv::Affine3d pose_diff = original.inv() * weighted;
double rotation_error = cv::norm(pose_diff.rvec());
double translation_error = cv::norm(pose_diff.translation());

SPDLOG_INFO("Pose diff: rotation={:.4f}°, translation={:.4f}m",
    rotation_error * 180 / M_PI, translation_error);
```

#### Sanity Checks

**✅ Good signs:**
- Best method has highest weight
- Total inliers >50
- Confidence 0.5-0.95
- Weights sum to 1.0

**❌ Red flags:**
- Low confidence (<0.2) despite good inliers
- One method >99% weight
- All methods equal weight
- Confidence/best method mismatch

## Code Organization Patterns

### File Dependencies

```
estimator.h (defines structs)
  ↓
estimator.cpp (implements algorithms)
  ↓
slam_thread.cpp (integrates into pipeline)
```

### Struct Evolution

- **Initial:** `pose, confidence`
- **After fusion:** Add weights, best_method
- **With covariance:** Add uncertainty fields
- **Future:** Kalman state, BA params

## Testing Checklist

When implementing similar features:

- [ ] Verify covariance valid only with ≥threshold inliers
- [ ] Check high-weight methods produce lower uncertainty
- [ ] Ensure translation_std in 0.001-5.0m range
- [ ] Ensure rotation_std in 0.0001-1.0 rad range
- [ ] Test fallback triggers at expected thresholds
- [ ] Verify logging shows covariance for all frames
- [ ] Profile that computation is <1ms

## Build System Lessons

### String Matching in Replacements

- Include 3-5 lines before/after target text
- Match whitespace exactly (spaces, tabs, newlines)
- Check namespace closing conventions (`}}` vs `}`)
- Use `od -c` to investigate format issues

### CMake Build Commands

```bash
# Build with Ninja, 4 parallel jobs
cmake --build .build/Debug -j 4

# Check for errors
grep -E "error|warning:" build.log

# Force full rebuild
cmake --build --clean-first .build/Debug
```

**Note:** Exit code 0 doesn't guarantee rebuild (Ninja may skip unchanged files).

## Future Enhancements

### Short Term
- Store covariance in `frame::estimated` for trajectory tracking
- Use covariance in motion prediction weighting
- Add Bundle Adjustment with covariance as preconditioner

### Medium Term
- Kalman Filtering with state covariance
- Outlier rejection via Mahalanobis distance
- Adaptive RANSAC with uncertainty-scaled thresholds

### Long Term
- Full SE(3) covariance with cross-terms
- Multi-frame trajectory optimization
- Sensor fusion with covariance weighting

## Summary

**Key principle:** *Uncertainty should drive decision-making*

- High uncertainty → use fallback
- Low uncertainty → trust fusion
- Quantitative bounds enable intelligent fallback
- Foundation for probabilistic SLAM
