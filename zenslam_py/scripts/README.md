# ZenSLAM Frame Statistics Analysis

This directory contains Python scripts for analyzing the frame processing statistics output by ZenSLAM.

## Overview

ZenSLAM now outputs comprehensive statistics for each frame to `frame_data.csv` including:

- **Timing metrics**: Detailed breakdown of processing time for each stage (detection, matching, tracking, estimation)
- **Feature counts**: Number of keypoints detected, tracked, matched, and triangulated
- **Quality metrics**: KLT success rate, feature response values, grid occupancy, track ages
- **Configuration info**: Detector, descriptor, and matcher types used

## Installation

Install the required Python packages:

```bash
cd zenslam_py/scripts
pip install -r requirements.txt
```

## Scripts

### 1. analyze_frame_stats.py

Identifies problematic frames where tracking fails or performance degrades.

**Usage:**
```bash
./analyze_frame_stats.py <path_to_frame_data.csv> [options]
```

**Options:**
- `--klt-threshold FLOAT`: KLT success rate threshold (default: 0.3)
- `--match-threshold INT`: Minimum feature matches (default: 20)
- `--triangulation-threshold INT`: Minimum triangulated points (default: 10)
- `--inlier-threshold INT`: Minimum pose inliers (default: 10)
- `--output PATH`: Output path for problem frames CSV

**Example:**
```bash
./analyze_frame_stats.py output/frame_data.csv --match-threshold 30
```

**Output:**
- Console summary of overall statistics and problem frames
- CSV file with problematic frames (`problems_frame_data.csv`)

### 2. plot_detector_comparison.py

Compares performance across different detector configurations by analyzing multiple runs.

**Usage:**
```bash
./plot_detector_comparison.py <csv_file1> <csv_file2> ... [options]
```

**Options:**
- `--output-dir PATH`: Output directory for plots (default: `detector_comparison/`)

**Example:**
```bash
./plot_detector_comparison.py \
    fast_orb/frame_data.csv \
    orb_orb/frame_data.csv \
    sift_sift/frame_data.csv \
    --output-dir comparison_results
```

**Output:**
- `detector_comparison.csv`: Comparison table
- `feature_comparison.png`: Feature counts over time
- `timing_comparison.png`: Timing breakdown comparison
- `distribution_comparison.png`: Box plots of key metrics

### 3. plot_timing_breakdown.py

Visualizes the timing breakdown of each processing stage.

**Usage:**
```bash
./plot_timing_breakdown.py <path_to_frame_data.csv> [options]
```

**Options:**
- `--output-dir PATH`: Output directory for plots (default: `timing_analysis/`)

**Example:**
```bash
./plot_timing_breakdown.py output/frame_data.csv
```

**Output:**
- `timing_stacked.png`: Stacked area plot of timing components
- `timing_detailed.png`: Detailed timing for each stage
- `timing_pie.png`: Average time distribution pie chart
- `timing_histogram.png`: Distribution of processing times

### 4. plot_feature_metrics.py

Visualizes feature detection, matching, and tracking metrics.

**Usage:**
```bash
./plot_feature_metrics.py <path_to_frame_data.csv> [options]
```

**Options:**
- `--output-dir PATH`: Output directory for plots (default: `feature_analysis/`)

**Example:**
```bash
./plot_feature_metrics.py output/frame_data.csv
```

**Output:**
- `feature_metrics.png`: Feature counts and KLT success rate over time
- `quality_metrics.png`: Response values, grid occupancy, track ages
- `correlation_analysis.png`: Scatter plots showing metric correlations

## Workflow Examples

### Analyzing a Single Run

```bash
# 1. Run ZenSLAM (outputs to output/frame_data.csv)
./zenslam_app --config tumvi.yaml --folder /path/to/dataset

# 2. Identify problem frames
./analyze_frame_stats.py output/frame_data.csv

# 3. Visualize timing breakdown
./plot_timing_breakdown.py output/frame_data.csv

# 4. Visualize feature metrics
./plot_feature_metrics.py output/frame_data.csv
```

### Comparing Detector Configurations

```bash
# Run with different detectors
./zenslam_app --config fast_orb.yaml --folder dataset --output fast_output
./zenslam_app --config orb_orb.yaml --folder dataset --output orb_output
./zenslam_app --config sift_sift.yaml --folder dataset --output sift_output

# Compare results
./plot_detector_comparison.py \
    fast_output/frame_data.csv \
    orb_output/frame_data.csv \
    sift_output/frame_data.csv
```

### Finding Tracking Failures

```bash
# Find frames with severe tracking issues
./analyze_frame_stats.py output/frame_data.csv \
    --klt-threshold 0.2 \
    --match-threshold 15 \
    --output severe_problems.csv

# The output CSV contains timestamps of problem frames
# You can then inspect those specific frames in your dataset
```

## CSV Output Format

The `frame_data.csv` file contains the following columns:

**Timing (in seconds):**
- `timestamp`, `t_wait`, `t_preprocessing`, `t_tracking`, `t_estimation`, `t_total`
- `t_detection_left`, `t_detection_right`, `t_klt`, `t_matching`, `t_triangulation`

**Feature Counts:**
- `n_keypoints_l`, `n_keypoints_r`, `n_tracked_l`, `n_tracked_r`, `n_new_l`, `n_new_r`
- `n_matches`, `n_triangulated`
- `n_3d3d`, `n_3d2d`, `n_2d2d`, `n_3d3d_inliers`, `n_3d2d_inliers`, `n_2d2d_inliers`

**Quality Metrics:**
- `klt_error_mean`, `klt_error_std`, `klt_success_rate`
- `match_distance_mean`, `match_distance_std`
- `epipolar_error_mean`, `fundamental_inliers`
- `response_mean_l`, `response_mean_r`, `response_std_l`, `response_std_r`
- `grid_occupancy_l`, `grid_occupancy_r`
- `track_age_mean`, `track_age_max`

**Configuration:**
- `detector_type`, `descriptor_type`, `matcher_type`

## Tips

1. **Real-time performance**: Check if `t_total < 33.3ms` (30 FPS) or `< 50ms` (20 FPS)
2. **Tracking health**: KLT success rate should generally be > 30-40%
3. **Minimum matches**: At least 20-30 stereo matches recommended for robust pose estimation
4. **Detector comparison**: Compare both accuracy (matches, inliers) and speed (timing)
5. **Problem identification**: Use `analyze_frame_stats.py` to automatically find frames needing attention

## Troubleshooting

**Import errors:**
```bash
pip install --upgrade pandas matplotlib seaborn
```

**Permission denied:**
```bash
chmod +x *.py
```

**No plots displayed:**
The scripts save plots to files instead of displaying them. Check the `--output-dir`.
