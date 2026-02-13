# Weighted Pose Fusion Implementation

## Overview
Implemented a **weighted pose fusion** system that intelligently combines multiple pose estimation methods instead of simply selecting the single best one. This improves robustness and accuracy in challenging scenarios.

## What Changed

### 1. New Data Structure: `weighted_pose_result` 
**File:** [zenslam_core/include/zenslam/estimator.h](zenslam_core/include/zenslam/estimator.h)

Added a new result struct to represent the fused pose:
```cpp
struct weighted_pose_result {
    cv::Affine3d pose;                    // Fused pose estimate
    double       confidence { 0.0 };      // Overall confidence (0-1)
    int          total_inliers { 0 };     // Total inliers across all methods
    
    // Individual method contributions (weights)
    double       weight_3d3d { 0.0 };
    double       weight_3d2d { 0.0 };
    double       weight_2d2d { 0.0 };
    double       weight_3d3d_lines { 0.0 };
    double       weight_3d2d_lines { 0.0 };
    
    // Best contributing method info
    std::string  best_method;
    size_t       best_method_inliers { 0 };
};
```

### 2. New Method: `estimate_pose_weighted()`
**File:** [zenslam_core/source/estimator.cpp](zenslam_core/source/estimator.cpp)

Computes confidence weights for each pose estimate based on:

#### Weight Calculation
For each pose method, confidence is computed as:
```
confidence = (inlier_ratio * 0.6 + error_quality * 0.3) * type_weight
```

Where:
- **inlier_ratio**: `inliers / (inliers + outliers)` - ratio of good matches
- **error_quality**: `exp(-mean_error / error_scale)` - exponential decay based on reprojection error
- **error_scale**: 
  - 3D methods (3D-3D): 0.05 meters
  - 2D methods (3D-2D, 2D-2D): 5.0 pixels
  - Lines: 0.8 weight reduction
- **type_weight**: 
  - Point-based methods: 1.0
  - Line-based methods: 0.8 (slightly less reliable)

#### Pose Fusion
- **Translation**: Weighted average across all methods
- **Rotation**: Weighted average of rotation vectors, converted back to rotation matrix
  - Converts each rotation matrix to axis-angle (rotation vector)
  - Computes weighted average of vectors
  - Converts averaged vector back to rotation matrix
  - Preserves rotation consistency

### 3. Updated SLAM Thread
**File:** [zenslam_core/source/slam_thread.cpp](zenslam_core/source/slam_thread.cpp)

Modified the pose estimation step to:
1. Compute all 5 pose estimates (original behavior preserved)
2. Apply weighted fusion to combine them
3. Log detailed information about contributions from each method
4. Fall back to motion prediction only if confidence is very low

**New logic:**
```cpp
auto estimate_result = estimator.estimate_pose(system[0], tracked);
auto weighted_result = estimator.estimate_pose_weighted(estimate_result);

// Use weighted pose if confidence is sufficient
cv::Affine3d chosen_pose = weighted_result.pose;
if (weighted_result.confidence < 0.1 || weighted_result.total_inliers < 5)
{
    SPDLOG_WARN("Low pose confidence, falling back to prediction");
    chosen_pose = pose_predicted;
}
```

## Benefits

1. **Robustness**: Uses information from all methods, not just the best one
2. **Confidence Quantification**: Provides a confidence score (0-1) for each pose
3. **Intelligent Fallback**: Better handling of low-feature frames
4. **Transparency**: Logs contribution of each method for debugging
5. **Flexibility**: Easy to adjust weighting strategy if needed

## Output/Logging

The fusion now logs:
- Each method's individual confidence weight
- Which method contributed most
- Total inliers across all methods
- Overall fusion confidence
- Detailed pose information

Example log output:
```
========== POSE ESTIMATION RESULTS ==========
Weighted Fusion Method:
  Best method: 3D-3D Points
  Best method inliers: 42
  Total inliers (all methods): 127
  Overall confidence: 0.876
  Method weights:
    3D-3D Points:   0.456
    3D-2D Points:   0.321
    2D-2D Points:   0.145
    3D-3D Lines:    0.078
    3D-2D Lines:    0.000
  Fused pose:
    [rotation matrix and translation vector]
================================================
```

## Configuration

The weighting strategy can be easily adjusted in the `compute_pose_weight()` function:
- Adjust component weights (currently 0.6/0.3 for inlier_ratio/error_quality)
- Change error scales based on sensor characteristics
- Modify type weights for different feature reliability

## Compilation

✅ Successfully compiles with:
- CMake + Ninja
- C++2b standards
- OpenCV 4.12.0
- All existing dependencies

## Future Enhancements

Potential improvements:
1. **Pose Covariance Estimation** - Add uncertainty quantification
2. **Temporal Filtering** - Add Kalman filter for temporal smoothness
3. **Adaptive Thresholds** - Adjust RANSAC parameters based on confidence
4. **Feature-Specific Weights** - Different weighting for FAST, ORB, etc.
5. **Motion Consistency Check** - Penalize poses inconsistent with motion model
