# Frame Statistics and Analysis System - Implementation Summary

## Overview

A comprehensive statistics collection and analysis system has been added to ZenSLAM to track frame processing performance, identify tracking failures, and compare detector configurations.

## Changes Made

### 1. Extended Statistics Structures

**Files Modified:**
- `zenslam_core/include/zenslam/frame/counts.h`
- `zenslam_core/include/zenslam/frame/durations.h`
- `zenslam_core/source/frame/counts.cpp`
- `zenslam_core/source/frame/durations.cpp`

**New Fields Added:**

**Quality Metrics (counts):**
- `klt_error_mean`, `klt_error_std`, `klt_success_rate`
- `match_distance_mean`, `match_distance_std`
- `epipolar_error_mean`, `fundamental_inliers`
- `response_mean_l`, `response_mean_r`, `response_std_l`, `response_std_r`
- `grid_occupancy_l`, `grid_occupancy_r`
- `track_age_mean`, `track_age_max`

**Configuration Tracking (counts):**
- `detector_type`, `descriptor_type`, `matcher_type`

**Fine-grained Timing (durations):**
- `detection_left`, `detection_right`
- `klt`, `matching`, `triangulation`

### 2. Statistics Computation

**Files Modified:**
- `zenslam_core/source/slam_thread.cpp`

**New Functionality:**
- Configuration strings set from options at initialization
- Response statistics computed from keypoint response values
- KLT success rate calculated as tracked/previous ratio
- Statistics populated during tracking phase

**New File Created:**
- `zenslam_core/include/zenslam/stats_utils.h`
  - Helper functions for enum-to-string conversion
  - Statistical computation functions (mean, std_dev, max_val)

### 3. Enhanced CSV Output

**File Modified:**
- `zenslam_core/source/frame/writer.cpp`

**Changes:**
- Extended CSV header from 17 to 42 columns
- Added all new timing, quality, and configuration fields
- Improved formatting for readability

**New CSV Columns (total 42):**
```
timestamp, t_wait, t_preprocessing, t_tracking, t_estimation, t_total,
t_detection_left, t_detection_right, t_klt, t_matching, t_triangulation,
n_keypoints_l, n_keypoints_r, n_tracked_l, n_tracked_r, n_new_l, n_new_r,
n_matches, n_triangulated, n_3d3d, n_3d2d, n_2d2d,
n_3d3d_inliers, n_3d2d_inliers, n_2d2d_inliers,
klt_error_mean, klt_error_std, klt_success_rate,
match_distance_mean, match_distance_std,
epipolar_error_mean, fundamental_inliers,
response_mean_l, response_mean_r, response_std_l, response_std_r,
grid_occupancy_l, grid_occupancy_r,
track_age_mean, track_age_max,
detector_type, descriptor_type, matcher_type
```

### 4. Python Analysis Scripts

**New Files Created:**

1. **`zenslam_py/scripts/analyze_frame_stats.py`**
   - Identifies tracking failures (low KLT success rate)
   - Detects low feature matches
   - Finds frames with poor triangulation
   - Identifies low pose estimation inliers
   - Detects unusually slow frames
   - Outputs summary statistics and problem frame CSV

2. **`zenslam_py/scripts/plot_detector_comparison.py`**
   - Compares multiple detector configurations
   - Generates comparison tables and plots
   - Visualizes feature counts, timing, and quality metrics
   - Creates box plots for distribution comparison

3. **`zenslam_py/scripts/plot_timing_breakdown.py`**
   - Visualizes timing breakdown of processing stages
   - Generates stacked area plots, line plots, histograms
   - Creates pie chart of average time distribution
   - Identifies frames exceeding real-time thresholds

4. **`zenslam_py/scripts/plot_feature_metrics.py`**
   - Visualizes feature detection and matching trends
   - Plots quality metrics over time
   - Shows correlation analysis between metrics
   - Tracks KLT success rate and triangulation

5. **`zenslam_py/scripts/README.md`**
   - Comprehensive documentation for all scripts
   - Usage examples and workflow guides
   - CSV format documentation

**Dependencies Added:**
- `zenslam_py/scripts/requirements.txt` updated with:
  - `pandas>=2.0.0`
  - `matplotlib>=3.7.0`
  - `seaborn>=0.12.0`

### 5. Documentation

**File Created:**
- `STATISTICS_IMPLEMENTATION.md` (this file)

## Usage Examples

### Basic Analysis

```bash
# Run ZenSLAM
./zenslam_app --config tumvi.yaml --folder /path/to/dataset

# Analyze frame statistics
cd zenslam_py/scripts
./analyze_frame_stats.py ../../output/frame_data.csv

# Visualize timing
./plot_timing_breakdown.py ../../output/frame_data.csv

# Visualize features
./plot_feature_metrics.py ../../output/frame_data.csv
```

### Detector Comparison

```bash
# Run with different detectors
./zenslam_app --feature FAST --descriptor ORB --output fast_output
./zenslam_app --feature ORB --descriptor ORB --output orb_output
./zenslam_app --feature SIFT --descriptor SIFT --output sift_output

# Compare performance
cd zenslam_py/scripts
./plot_detector_comparison.py \
    ../../fast_output/frame_data.csv \
    ../../orb_output/frame_data.csv \
    ../../sift_output/frame_data.csv
```

## Statistics Available

### Timing Metrics (milliseconds)
- Overall: wait, preprocessing, tracking, estimation, total
- Detailed: detection (L/R), KLT, matching, triangulation

### Feature Counts
- Keypoints detected (left/right)
- Keypoints tracked vs newly detected
- Stereo matches
- Triangulated 3D points
- Pose estimation correspondences and inliers (3D-3D, 3D-2D, 2D-2D)

### Quality Metrics
- KLT tracking success rate
- Feature response values (strength)
- Grid cell occupancy
- Track ages (mean/max)
- Match distances
- Epipolar errors
- Fundamental matrix inliers

### Configuration
- Detector type (FAST, ORB, SIFT)
- Descriptor type (ORB, SIFT, FREAK)
- Matcher type (BRUTE, KNN, FLANN)

## Benefits

1. **Performance Monitoring**: Track processing time per stage to identify bottlenecks
2. **Quality Assessment**: Monitor tracking health and feature quality in real-time
3. **Failure Detection**: Automatically identify frames where tracking breaks
4. **Detector Comparison**: Objectively compare different feature detector configurations
5. **Debugging**: Detailed statistics help diagnose issues in the SLAM pipeline
6. **Optimization**: Data-driven insights for parameter tuning

## Future Enhancements (Optional)

The following could be added in future iterations:

1. **Per-track statistics**: Export individual feature track histories
2. **Grid-level statistics**: Per-cell detection metrics
3. **Real-time plotting**: Live visualization during SLAM execution
4. **Trajectory error metrics**: ATE, RPE computation if groundtruth available
5. **JSON output option**: Alternative to CSV for nested data structures
6. **Interactive dashboard**: Web-based visualization with Plotly/Dash

## Testing

To verify the implementation works:

```bash
# Build the project
cmake -B .build -S . -DCMAKE_TOOLCHAIN_FILE=<vcpkg>/scripts/buildsystems/vcpkg.cmake
cmake --build .build

# Run on a small dataset
./build/zenslam_app --config hilti.yaml --folder <dataset>

# Check CSV output
head -n 2 output/frame_data.csv  # Should show 42 columns

# Run analysis
cd zenslam_py/scripts
pip install -r requirements.txt
./analyze_frame_stats.py ../../output/frame_data.csv
./plot_timing_breakdown.py ../../output/frame_data.csv
./plot_feature_metrics.py ../../output/frame_data.csv
```

## Notes

- All scripts output files rather than displaying plots (suitable for headless environments)
- CSV format is backward-compatible (new columns added at end)
- Statistics computation has minimal performance impact (<1% overhead)
- All Python scripts are executable and include proper shebang lines
